// main.c — Pico W Code-39 barcode test (GP22 D0, USB CDC 115200)
// - No Wi-Fi/MQTT needed
// - Full Code-39 table (+ start/stop '*')
// - Real-time edge debounce + post de-glitch merge
// - Strict guard handling (must see '*' ... '*')
// - Heartbeat + decoded output (or concise reason)
// - Fixes: clears output on finalize; no ghost chars when under-threshold

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"

// ===== Wiring / behavior =====
#define BAR_D0_GPIO             22    // Barcode D0 on GP22
#define BAR_ACTIVE_LOW           1    // 1: module pulls LOW on black; 0: HIGH on black

// ===== Debug toggles =====
#define BAR_DEBUG_EDGES          0    // 1: print each committed segment
#define BAR_DEBUG_NO_DECODE      1    // 1: print summary when decode fails
#define BAR_DEBUG_DUMP           0    // 1: dump segments & threshold classification

// ===== Timing / decode params (tweakable knobs) =====
#define BARCODE_IDLE_TIMEOUT_MS 3500  // finalize after this long with no edges
#define BARCODE_MAX_ELEMS        512  // collected segments (bars+spaces)
#define BARCODE_MIN_RATIO_WIDE  1.35f // tolerant narrow/wide bounds
#define BARCODE_MAX_RATIO_WIDE  4.50f
#define MIN_SEG_US               900  // real-time debounce: ignore flips < 0.9 ms
#define MIN_ELEMS_FOR_DECODE      27  // start + 1 data + stop (~27–31 elems typical)

// ===== Capture buffer =====
typedef struct {
    uint64_t t_last;
    int      last_level;               // -1 unknown, 0=space(white), 1=bar(black)
    uint32_t dur_us[BARCODE_MAX_ELEMS];
    uint8_t  lvl[BARCODE_MAX_ELEMS];   // 1=bar, 0=space
    int      n;
    uint64_t pass_last_edge_us;
} cap_t;

static cap_t g = {0};

// ---------- utils ----------
static inline void cap_reset(void) {
    memset(&g, 0, sizeof(g));
    g.last_level = -1;
}

static float median_u32(const uint32_t *a, int n) {
    if (n<=0) return 0.f;
    uint32_t buf[BARCODE_MAX_ELEMS];
    int m = (n>BARCODE_MAX_ELEMS)?BARCODE_MAX_ELEMS:n;
    for (int i=0;i<m;i++) buf[i]=a[i];
    for (int i=1;i<m;i++){uint32_t v=buf[i];int j=i-1;while(j>=0&&buf[j]>v){buf[j+1]=buf[j];j--;}buf[j+1]=v;}
    return (m&1)? (float)buf[m/2] : 0.5f*(buf[m/2-1]+buf[m/2]);
}

static float choose_thr_us(const uint32_t *dur, int n) {
    if (n<=0) return 0.f;
    uint32_t buf[BARCODE_MAX_ELEMS];
    int m = (n>BARCODE_MAX_ELEMS)?BARCODE_MAX_ELEMS:n;
    for (int i=0;i<m;i++) buf[i]=dur[i];
    for (int i=1;i<m;i++){uint32_t v=buf[i];int j=i-1;while(j>=0&&buf[j]>v){buf[j+1]=buf[j];j--;}buf[j+1]=v;}
    int nlow=m*2/3; if(nlow<1)nlow=1;
    int nhigh=m/3;  if(nhigh<1)nhigh=1;
    float narrow=median_u32(buf,nlow);
    float wide  =median_u32(buf+(m-nhigh),nhigh);
    if (wide < narrow*BARCODE_MIN_RATIO_WIDE) wide = narrow*BARCODE_MIN_RATIO_WIDE;
    if (wide > narrow*BARCODE_MAX_RATIO_WIDE) wide = narrow*BARCODE_MAX_RATIO_WIDE;
    return 0.5f*(narrow + wide);
}

// ---------- post de-glitch merge ----------
static int filter_segments(const uint32_t *in_dur, const uint8_t *in_lvl, int n,
                           uint32_t *out_dur, uint8_t *out_lvl) {
    if (n <= 0) return 0;

    // Estimate a "narrow-ish" duration to set a dynamic glitch threshold
    uint32_t tmp[BARCODE_MAX_ELEMS];
    int m = n > BARCODE_MAX_ELEMS ? BARCODE_MAX_ELEMS : n;
    for (int i=0;i<m;i++) tmp[i] = in_dur[i];
    for (int i=1;i<m;i++){ uint32_t v=tmp[i]; int j=i-1; while (j>=0 && tmp[j]>v) { tmp[j+1]=tmp[j]; j--; } tmp[j+1]=v; }
    int low = m*2/3; if (low < 1) low = 1;
    uint32_t narrow = tmp[low/2];
    uint32_t GLITCH_US = narrow/4;
    if (GLITCH_US < 800)  GLITCH_US = 800;
    if (GLITCH_US > 8000) GLITCH_US = 8000;

    int out_n = 0;
    uint32_t acc = in_dur[0];
    uint8_t  lvl = in_lvl[0];

    for (int i=1;i<n;i++) {
        uint32_t d = in_dur[i];
        uint8_t  l = in_lvl[i];

        if (d < GLITCH_US) { acc += d; continue; }   // absorb micro-blip
        if (l == lvl)      { acc += d; continue; }   // merge same level

        // level change with decent duration → flush
        if (out_n < BARCODE_MAX_ELEMS) {
            out_dur[out_n] = acc;
            out_lvl[out_n] = lvl;
            out_n++;
        }
        acc = d; lvl = l;
    }
    if (out_n < BARCODE_MAX_ELEMS) { out_dur[out_n] = acc; out_lvl[out_n] = lvl; out_n++; }
    return out_n;
}

// ---------- real-time edge debounce + capture ----------
static inline void on_edge(int level_now, uint64_t now_us) {
    if (g.last_level < 0) {
        g.last_level = level_now;
        g.t_last = now_us;
        g.pass_last_edge_us = now_us;
        return;
    }

    uint32_t dt = (uint32_t)(now_us - g.t_last);

    // Real-time debounce: swallow very short flips
    if (dt < MIN_SEG_US) {
        // treat as noise: extend current run; do NOT toggle level
        g.t_last = now_us;
        g.pass_last_edge_us = now_us;
        return;
    }

    if (g.n < BARCODE_MAX_ELEMS) {
        g.dur_us[g.n] = dt;
        g.lvl[g.n]    = (uint8_t)g.last_level;
        g.n++;
#if BAR_DEBUG_EDGES
        printf("[EDGE] lvl=%u dt_us=%u\n", (unsigned)g.last_level, (unsigned)dt);
#endif
    }
    g.last_level = level_now;
    g.t_last = now_us;
    g.pass_last_edge_us = now_us;
}

// ---------- Full Code 39 map (43 symbols + '*') ----------
typedef struct { char ch; unsigned short pat9; } c39map_t;
static const c39map_t C39[] = {
    {'0',0b000110100},{'1',0b100100001},{'2',0b001100001},{'3',0b101100000},
    {'4',0b000110001},{'5',0b100110000},{'6',0b001110000},{'7',0b000100101},
    {'8',0b100100100},{'9',0b001100100},
    {'A',0b100001001},{'B',0b001001001},{'C',0b101001000},{'D',0b000011001},
    {'E',0b100011000},{'F',0b001011000},{'G',0b000001101},{'H',0b100001100},
    {'I',0b001001100},{'J',0b000011100},
    {'K',0b100000011},{'L',0b001000011},{'M',0b101000010},{'N',0b000010011},
    {'O',0b100010010},{'P',0b001010010},{'Q',0b000000111},{'R',0b100000110},
    {'S',0b001000110},{'T',0b000010110},
    {'U',0b110000001},{'V',0b011000001},{'W',0b111000000},{'X',0b010010001},
    {'Y',0b110010000},{'Z',0b011010000},
    {'-',0b010000101},{'.',0b110000100},{' ',0b011000100},
    {'$',0b010101000},{'/',0b010100010},{'+',0b010001010},{'%',0b000101010},
    {'*',0b010010100} // start/stop
};
static inline int map_find(unsigned short p){
    for (unsigned i=0;i<sizeof(C39)/sizeof(C39[0]);++i)
        if (C39[i].pat9 == p) return (int)i;
    return -1;
}
static inline unsigned short pack9(const uint8_t *w){
    unsigned short v=0; for(int i=0;i<9;i++) v=(unsigned short)((v<<1)|(w[i]?1:0)); return v;
}

// ---------- optional dump ----------
static void dump_pass(const uint32_t *dur, const uint8_t *lvl, int n, float thr) {
#if BAR_DEBUG_DUMP
    printf("[DUMP] n=%d thr=%.1f us\n", n, thr);
    int shown = n < 80 ? n : 80;
    for (int i = 0; i < shown; i++) {
        printf("  %02d: lvl=%d dt=%u us%s\n", i, lvl[i], (unsigned)dur[i],
               (dur[i] > (uint32_t)thr) ? "  (WIDE)" : "");
    }
    if (n > shown) printf("  ... (%d more)\n", n - shown);
#else
    (void)dur; (void)lvl; (void)n; (void)thr;
#endif
}

// ---------- finalize & decode ----------
static bool try_finalize(uint64_t now_us, char *out, int out_sz,
                         float *conf_out, float *thr_out, int *elems_out)
{
    // Clear outputs so partial passes can't leak bogus characters
    if (out && out_sz > 0) out[0] = '\0';
    if (conf_out) *conf_out = 0.0f;
    if (thr_out)  *thr_out  = 0.0f;

    if (!g.pass_last_edge_us) return false;
    uint32_t idle_ms = (uint32_t)((now_us - g.pass_last_edge_us)/1000u);
    if (idle_ms < BARCODE_IDLE_TIMEOUT_MS) return false;

    // close trailing segment (respect debounce)
    if (g.last_level >= 0 && g.n < BARCODE_MAX_ELEMS) {
        uint32_t dt = (uint32_t)(now_us - g.t_last);
        if (dt >= MIN_SEG_US) {
            g.dur_us[g.n] = dt; g.lvl[g.n] = (uint8_t)g.last_level; g.n++;
#if BAR_DEBUG_EDGES
            printf("[EDGE] lvl=%u dt_us=%u (final)\n",(unsigned)g.last_level,(unsigned)dt);
#endif
        }
    }

    if (elems_out) *elems_out = g.n;

    // Not enough segments? Just report and bail (caller prints "too-few-segments")
    if (g.n < MIN_ELEMS_FOR_DECODE) { cap_reset(); return true; }

    // Post-filter / merge
    uint32_t f_dur[BARCODE_MAX_ELEMS];
    uint8_t  f_lvl[BARCODE_MAX_ELEMS];
    int fn = filter_segments(g.dur_us, g.lvl, g.n, f_dur, f_lvl);
    if (fn < MIN_ELEMS_FOR_DECODE) { cap_reset(); return true; }

    // Threshold using filtered segments
    float thr = choose_thr_us(f_dur, fn);
    if (thr_out) *thr_out = thr;

    dump_pass(f_dur, f_lvl, fn, thr);

    uint8_t wide[BARCODE_MAX_ELEMS];
    for (int i=0;i<fn;i++) wide[i] = (f_dur[i] > thr) ? 1 : 0;

    // Skip leading spaces
    int i=0; while (i<fn && f_lvl[i]==0) i++;

    // Strict guards: must see '*' ... '*'
    bool seen_start = false, seen_stop = false;
    char buf[128]; int blen = 0; int chars = 0; int recognized = 0;

    while (i + 9 <= fn) {
        // must start with BAR and alternate BAR/SPACE
        bool ok=true;
        for (int k=0;k<9;k++){
            int expected = (k%2==0)?1:0;
            if (f_lvl[i+k] != expected){ ok=false; break; }
        }
        if (!ok) { i++; continue; }

        unsigned short p = pack9(&wide[i]);
        int idx = map_find(p);
        char ch = (idx >= 0) ? C39[idx].ch : '?';

        if (!seen_start) {
            if (ch == '*') { seen_start = true; recognized++; }
            i += (i + 9 < fn && f_lvl[i + 9] == 0) ? 10 : 9;
            continue;
        }

        if (ch == '*') { seen_stop = true; recognized++; break; }

        // Between guards: accumulate payload
        if (blen < (int)sizeof(buf) - 1) buf[blen++] = ch;
        chars++;

        // advance (consume inter-character narrow space if present)
        i += (i + 9 < fn && f_lvl[i + 9] == 0) ? 10 : 9;
    }

    buf[blen] = '\0';

    bool success = (seen_start && seen_stop && blen > 0);
    if (out && out_sz > 0) {
        int cpy = (blen < out_sz - 1) ? blen : out_sz - 1;
        memcpy(out, buf, cpy); out[cpy] = '\0';
    }
    if (conf_out) {
        // crude confidence: recognized symbols / (payload chars + 2 guards)
        *conf_out = (success && (chars + 2) > 0) ? (float)recognized / (float)(chars + 2) : 0.0f;
    }

    cap_reset();
    return true;  // finalized; caller prints result or diag
}

// ========================= MAIN =========================
int main(void){
    stdio_init_all();
    sleep_ms(400);
    printf("=== Pico W Code-39 Test (GP22 D0, USB 115200) ===\n");
    printf("Move PERPENDICULAR to bars. Start/end in white. Height 1–2 mm. 5–8 cm/s.\n");

    gpio_init(BAR_D0_GPIO);
    gpio_set_dir(BAR_D0_GPIO, false);
#if BAR_ACTIVE_LOW
    gpio_pull_up(BAR_D0_GPIO);
#else
    gpio_pull_down(BAR_D0_GPIO);
#endif

    int prev = -1;
    uint32_t last_hb = 0;

    while (true) {
        uint64_t now = time_us_64();

        // Read D0 and normalize to 1=bar(black), 0=space(white)
        int raw = gpio_get(BAR_D0_GPIO);
        int level = (
#if BAR_ACTIVE_LOW
            (raw == 0)
#else
            (raw == 1)
#endif
        ) ? 1 : 0;

        // Heartbeat so the terminal never looks blank
        uint32_t ms = to_ms_since_boot(get_absolute_time());
        if (ms - last_hb >= 1000) {
            printf("[HB] raw=%d (0=white,1=black)\n", level);
            last_hb = ms;
        }

        // Real-time debounced edge detection
        if (prev < 0) prev = level;
        if (level != prev) {
            on_edge(level, now);
            prev = level;
        }

        // Finalize pass after idle -> decode or concise report
        char decoded[128]; float conf=0.f; float thr=0.f; int elems=0;
        if (try_finalize(now, decoded, sizeof(decoded), &conf, &thr, &elems)) {
            bool has_char = (decoded[0] != '\0');
            if (has_char) {
                printf("[BAR39] \"%s\" conf=%.2f thr_us=%.1f elems=%d\n",
                       decoded, conf, thr, elems);
            } else {
#if BAR_DEBUG_NO_DECODE
                const char* why = (elems < MIN_ELEMS_FOR_DECODE) ? "too-few-segments" : "pattern/threshold";
                printf("[BAR39] no-decode (%s): thr_us=%.1f elems=%d conf=%.2f decoded=\"%s\"\n",
                       why, thr, elems, conf, decoded);
#endif
            }
        }

        sleep_ms(2);
    }
}
