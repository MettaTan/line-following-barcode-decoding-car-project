#include "barcode39.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

typedef struct {
    uint64_t t_last;         // last edge time
    int      last_level;     // -1 unknown, 0 white, 1 black
    // durations and levels of segments (bars+spaces), starting from first completed segment
    uint32_t dur_us[BARCODE_MAX_ELEMS];
    uint8_t  lvl[BARCODE_MAX_ELEMS];   // 1=bar(black), 0=space(white)
    int      n;

    // telemetry accumulators
    uint64_t pass_last_edge_us;
    uint64_t acc_black_us;
    uint64_t acc_white_us;
} barcap_t;

static barcap_t g = { .t_last = 0, .last_level = -1, .n = 0 };

// --- Public helpers ---
void barcode39_reset(void) {
    g.t_last = 0; g.last_level = -1; g.n = 0;
    g.pass_last_edge_us = 0;
    g.acc_black_us = g.acc_white_us = 0;
}

bool barcode39_is_capturing(void) {
    return g.n > 0;
}

void barcode39_get_duty_ms(float *black_ms, float *white_ms, float *duty_pct, uint32_t *age_ms) {
    float b = (float)g.acc_black_us / 1000.0f;
    float w = (float)g.acc_white_us / 1000.0f;
    float tot = b + w;
    if (black_ms) *black_ms = b;
    if (white_ms) *white_ms = w;
    if (duty_pct) *duty_pct = (tot > 0.f) ? (100.0f * b / tot) : 0.f;
    if (age_ms) *age_ms = g.pass_last_edge_us ? (uint32_t)((time_us_64() - g.pass_last_edge_us)/1000u) : 0u;
}

// --- Edge ingestion ---
void barcode39_on_edge(int level_now, uint64_t t_us) {
    if (g.last_level < 0) { // first observation; just prime the state
        g.last_level = level_now;
        g.t_last = t_us;
        g.pass_last_edge_us = t_us;
        return;
    }

    // Close previous segment
    if (g.n < BARCODE_MAX_ELEMS) {
        uint32_t dur = (uint32_t)(t_us - g.t_last);
        g.dur_us[g.n] = dur;
        g.lvl[g.n]    = (uint8_t)g.last_level;
        g.n++;
        if (g.last_level) g.acc_black_us += dur; else g.acc_white_us += dur;
    }

    // Begin next segment
    g.last_level = level_now;
    g.t_last = t_us;
    g.pass_last_edge_us = t_us;
}

// --- Narrow/Wide classification utilities ---
static float median_u32(const uint32_t *a, int n) {
    // small n -> insertion sort copy
    uint32_t tmp[BARCODE_MAX_ELEMS];
    int m = (n > BARCODE_MAX_ELEMS) ? BARCODE_MAX_ELEMS : n;
    for (int i = 0; i < m; ++i) tmp[i] = a[i];
    for (int i = 1; i < m; ++i) { uint32_t v=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>v){tmp[j+1]=tmp[j]; j--;} tmp[j+1]=v; }
    if (m==0) return 0.f;
    if (m&1) return (float)tmp[m/2];
    return 0.5f*(float)(tmp[m/2-1]+tmp[m/2]);
}

// returns threshold(in us) splitting narrow/wide
static float choose_threshold_us(const uint32_t *dur, int n) {
    // Take lower half median as narrow; upper third median as wide.
    // Works well for Code39 (3 wide of 9 per char).
    if (n <= 0) return 0.f;
    uint32_t tmp[BARCODE_MAX_ELEMS];
    int m = (n > BARCODE_MAX_ELEMS) ? BARCODE_MAX_ELEMS : n;
    for (int i = 0; i < m; ++i) tmp[i] = dur[i];
    // sort
    for (int i = 1; i < m; ++i) { uint32_t v=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>v){tmp[j+1]=tmp[j]; j--;} tmp[j+1]=v; }
    int nlow = m*2/3; if (nlow<1) nlow=1;
    int nhigh = m/3;  if (nhigh<1) nhigh=1;
    float narrow = median_u32(tmp, nlow);
    float wide   = median_u32(tmp + (m - nhigh), nhigh);
    if (wide < narrow * BARCODE_MIN_RATIO_WIDE) wide = narrow * BARCODE_MIN_RATIO_WIDE;
    if (wide > narrow * BARCODE_MAX_RATIO_WIDE) wide = narrow * BARCODE_MAX_RATIO_WIDE;
    return 0.5f*(narrow + wide);
}

// --- Code39 mapping (bars+spaces; 1=wide, 0=narrow). 9 elements per char.
// For your track use we include the two commands you used: '*', 'A', 'Z'.
// Extend easily by appending entries { 'X', 9-bit-pattern }.
// The bit 8 is the FIRST element (bar), bit 0 the LAST (space).
typedef struct { char ch; unsigned short pattern9; } map_t;
static const map_t MAP_PARTIAL[] = {
    // NOTE: These are the canonical Code39 patterns for the start/stop '*' and for A, Z.
    // If you later want full decoding, drop in a complete 43-char table here.
    {'*', 0b010010100},   // * (start/stop)
    {'A', 0b100001001},   // A
    {'Z', 0b100100001},   // Z
};

static int find_map(unsigned short pat) {
    for (unsigned i=0;i<sizeof(MAP_PARTIAL)/sizeof(MAP_PARTIAL[0]);++i)
        if (MAP_PARTIAL[i].pattern9 == pat) return (int)i;
    return -1;
}

// Build a 9-bit pattern from 9 elements (w/n), MSB first.
// elems[i] = 1 if wide, 0 if narrow. The sequence starts with a BAR and alternates.
static unsigned short pack9(const uint8_t *elems) {
    unsigned short v = 0;
    for (int i=0;i<9;i++) v = (unsigned short)((v<<1) | (elems[i] ? 1 : 0));
    return v;
}

// Try to segment dur/lvl -> characters; write ASCII without guards into out.
bool barcode39_try_finalize(uint64_t now_us, char *out, int out_sz, float *confidence) {
    if (g.pass_last_edge_us == 0) return false;
    uint32_t idle_ms = (uint32_t)((now_us - g.pass_last_edge_us) / 1000u);
    if (idle_ms < BARCODE_IDLE_TIMEOUT_MS) return false; // pass not finished yet

    // Close the trailing open segment (up to now_us)
    if (g.last_level >= 0 && g.n < BARCODE_MAX_ELEMS) {
        uint32_t dur = (uint32_t)(now_us - g.t_last);
        g.dur_us[g.n] = dur;
        g.lvl[g.n]    = (uint8_t)g.last_level;
        g.n++;
        if (g.last_level) g.acc_black_us += dur; else g.acc_white_us += dur;
    }

    // Compute global threshold to split narrow vs wide.
    float thr = choose_threshold_us(g.dur_us, g.n);
    if (thr <= 0.f) { barcode39_reset(); return false; }

    // Convert to narrow(0)/wide(1)
    uint8_t nw[BARCODE_MAX_ELEMS];
    for (int i=0;i<g.n;i++) nw[i] = (g.dur_us[i] > thr) ? 1 : 0;

    // A valid Code39 char is 9 elements, starting with BAR (black), alt bar/space,
    // followed by a narrow inter-character SPACE (white). The overall stream may start
    // with a SPACE before the first BAR; skip leading spaces.
    int i = 0;
    while (i < g.n && g.lvl[i] == 0) i++;

    char buf[64]; int blen = 0;
    int guards_seen = 0;
    int chars_total = 0;

    while (i + 9 <= g.n) {
        // 9-element window: must alternate bar/space, starting with bar.
        bool ok = true;
        for (int k=0;k<9;k++) {
            int expected_lvl = (k % 2 == 0) ? 1 : 0; // 0,2,4,6,8 = bars
            if (g.lvl[i+k] != expected_lvl) { ok=false; break; }
        }
        if (!ok) { i++; continue; }

        // Pack 9 narrow/wide to a pattern
        unsigned short pat = pack9(&nw[i]);

        // Try map
        int idx = find_map(pat);
        if (idx < 0) { i++; continue; } // unknown; slide

        char ch = MAP_PARTIAL[idx].ch;
        if (ch == '*') {
            guards_seen++;
        } else {
            // data char (between two guards)
            if (blen < (int)sizeof(buf)-1) buf[blen++] = ch;
        }

        chars_total++;

        // Expect an inter-character narrow SPACE after each char, except maybe after final stop.
        int advance = 9;
        if (i + 9 < g.n) {
            // if there is a following element and it is a SPACE, consume exactly one space element
            if (g.lvl[i+9] == 0) advance = 10;
        }
        i += advance;

        // If we have seen both guards and we see end of stream, stop.
        if (guards_seen >= 2 && (i >= g.n - 2)) break;
    }

    bool success = (guards_seen >= 2 && blen > 0);
    if (success) {
        buf[blen] = '\0';
        // confidence heuristic: fraction of characters that mapped (we slid on unknowns)
        if (confidence) *confidence = fminf(1.f, (float)blen / (float)(chars_total > 0 ? chars_total : 1));
        if (out && out_sz > 0) {
            // copy to out
            int copy = (blen < out_sz-1) ? blen : (out_sz-1);
            memcpy(out, buf, copy); out[copy] = '\0';
        }
    }

    barcode39_reset();
    return success;
}
