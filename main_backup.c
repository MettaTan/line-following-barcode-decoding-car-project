#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/time.h"

#include "hardware/adc.h"

#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "line.h"

#include "mqtt_client.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"

// ===================== Pin Map (MUST MATCH WIRING) =====================
#define LF_ADC_GPIO        26      // GP26 / ADC0 (analog reference for PID)

// Barcode sensor (digital edges)
#define BAR_D0_GPIO        22      // GP27 (D0)

// Encoders
#define ENC_LEFT_GPIO       2      // GP2  (left)
#define ENC_RIGHT_GPIO      4      // GP4  (right)

// Motors (H-bridge)
#define M1A                 8
#define M1B                 9
#define M2A                10
#define M2B                11

// ===================== General Timing =====================
#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV      20
#endif

#define SAMPLE_IR_MS        2
#define PRINT_MS          250

// ===================== Line Following (ADC-based) =====================
#define LF_ENABLE           1
#define LF_BASE_SPEED_PCT  36.0f
#define LF_TURN_PCT        34.0f
#define LF_KP               0.0052f
#define LF_KD               0.0000f

#define LF_ERR_DEADBAND     15
#define IR_EMA_ALPHA        0.25f
#define LF_U_SMOOTH_ALPHA   0.12f
#define SLEW_PCT_PER_TICK    6.0f
#define MIN_WHEEL_PCT       18.0f

#ifndef LF_LOST_MS
#define LF_LOST_MS        450
#endif
#ifndef LF_RECOVER_PCT
#define LF_RECOVER_PCT    22.0f
#endif

// Optional “reverse on white”
#define WHITE_REV_ENABLE         0
#define WHITE_REV_MIN_MS       120
#define WHITE_REV_MAX_MS       280
#define WHITE_REV_PCT           18.0f
#define WHITE_REV_TURN_PCT      26.0f
#define WHITE_REV_SLEW           6.0f

// ===================== Barcode 39 (standards-correct) =====================
// Capture bars AND spaces; 9 elements per character; require * guards.
#ifndef BARCODE_IDLE_TIMEOUT_MS
#define BARCODE_IDLE_TIMEOUT_MS  2200   // end-of-pass idle timeout (ms)
#endif
#ifndef BARCODE_MAX_ELEMS
#define BARCODE_MAX_ELEMS         256   // bars+spaces in one pass
#endif
#ifndef BARCODE_MIN_RATIO_WIDE
#define BARCODE_MIN_RATIO_WIDE   1.6f
#endif
#ifndef BARCODE_MAX_RATIO_WIDE
#define BARCODE_MAX_RATIO_WIDE   3.6f
#endif

typedef struct {
    uint64_t t_last;                   // last edge timestamp
    int      last_level;               // -1 unknown, 0 white(space), 1 black(bar)
    uint32_t dur_us[BARCODE_MAX_ELEMS];
    uint8_t  lvl[BARCODE_MAX_ELEMS];   // 1=bar, 0=space
    int      n;

    // telemetry accumulators
    uint64_t pass_last_edge_us;
    uint64_t acc_black_us;
    uint64_t acc_white_us;
} barcode_cap_t;

static barcode_cap_t bcap = { .t_last = 0, .last_level = -1, .n = 0 };

// --- helpers for barcode telemetry
static inline void barcode_reset(void) {
    bcap.t_last = 0; bcap.last_level = -1; bcap.n = 0;
    bcap.pass_last_edge_us = 0;
    bcap.acc_black_us = bcap.acc_white_us = 0;
}
static inline bool barcode_is_capturing(void) { return bcap.n > 0; }
static inline void barcode_get_duty_ms(float *black_ms, float *white_ms, float *duty_pct, uint32_t *age_ms) {
    float b = (float)bcap.acc_black_us / 1000.0f;
    float w = (float)bcap.acc_white_us / 1000.0f;
    float tot = b + w;
    if (black_ms) *black_ms = b;
    if (white_ms) *white_ms = w;
    if (duty_pct) *duty_pct = (tot > 0.f) ? (100.0f * b / tot) : 0.f;
    if (age_ms) *age_ms = bcap.pass_last_edge_us ? (uint32_t)((time_us_64() - bcap.pass_last_edge_us)/1000u) : 0u;
}

// --- ingest each edge (1=bar, 0=space), at timestamp now_us
static inline void barcode_on_edge(int level_now, uint64_t now_us) {
    if (bcap.last_level < 0) {
        bcap.last_level = level_now;
        bcap.t_last = now_us;
        bcap.pass_last_edge_us = now_us;
        return;
    }

    if (bcap.n < BARCODE_MAX_ELEMS) {
        uint32_t dt = (uint32_t)(now_us - bcap.t_last);
        bcap.dur_us[bcap.n] = dt;
        bcap.lvl[bcap.n]    = (uint8_t)bcap.last_level;
        bcap.n++;

        // accumulate + publish the segment that just ended
        if (bcap.last_level) bcap.acc_black_us += dt; else bcap.acc_white_us += dt;
        mqtt_publish_barcode_edge(bcap.last_level, dt);
    }

    bcap.last_level = level_now;
    bcap.t_last = now_us;
    bcap.pass_last_edge_us = now_us;
}

// --- small statistics for thresholding
static float median_u32(const uint32_t *a, int n) {
    if (n <= 0) return 0.f;
    uint32_t tmp[BARCODE_MAX_ELEMS];
    int m = (n > BARCODE_MAX_ELEMS) ? BARCODE_MAX_ELEMS : n;
    for (int i=0;i<m;i++) tmp[i] = a[i];
    for (int i=1;i<m;i++){ uint32_t v=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>v){ tmp[j+1]=tmp[j]; j--; } tmp[j+1]=v; }
    if (m&1) return (float)tmp[m/2];
    return 0.5f*(float)(tmp[m/2-1] + tmp[m/2]);
}

static float choose_threshold_us(const uint32_t *dur, int n) {
    if (n <= 0) return 0.f;
    uint32_t tmp[BARCODE_MAX_ELEMS];
    int m = (n > BARCODE_MAX_ELEMS) ? BARCODE_MAX_ELEMS : n;
    for (int i=0;i<m;i++) tmp[i] = dur[i];
    for (int i=1;i<m;i++){ uint32_t v=tmp[i]; int j=i-1; while(j>=0 && tmp[j]>v){ tmp[j+1]=tmp[j]; j--; } tmp[j+1]=v; }

    int nlow = m*2/3; if (nlow<1) nlow=1;
    int nhigh = m/3;  if (nhigh<1) nhigh=1;
    float narrow = median_u32(tmp, nlow);
    float wide   = median_u32(tmp + (m - nhigh), nhigh);

    if (wide < narrow * BARCODE_MIN_RATIO_WIDE) wide = narrow * BARCODE_MIN_RATIO_WIDE;
    if (wide > narrow * BARCODE_MAX_RATIO_WIDE) wide = narrow * BARCODE_MAX_RATIO_WIDE;
    return 0.5f*(narrow + wide);
}

// --- minimal Code 39 map (extendable)
// 9 elements per char, bars/spaces alternating, MSB=first element.
// 1=wide, 0=narrow. Include '*' guards + A/Z (your field signals).
typedef struct { char ch; unsigned short pat9; } c39map_t;
static const c39map_t C39_MAP[] = {
    {'*', 0b010010100},  // start/stop
    {'A', 0b100001001},
    {'Z', 0b100100001},
};
static int c39_find(unsigned short pat) {
    for (unsigned i=0;i<sizeof(C39_MAP)/sizeof(C39_MAP[0]);++i)
        if (C39_MAP[i].pat9 == pat) return (int)i;
    return -1;
}
static unsigned short pack9(const uint8_t *nw) {
    unsigned short v=0; for (int i=0;i<9;i++) v=(unsigned short)((v<<1)|(nw[i]?1:0)); return v;
}

// Try finalize when idle; if success, returns true and writes decoded text (no guards)
static bool barcode_finalize_if_ready(uint64_t now_us, char *out, int out_sz, float *conf_out) {
    if (bcap.pass_last_edge_us == 0) return false;
    uint32_t idle_ms = (uint32_t)((now_us - bcap.pass_last_edge_us)/1000u);
    if (idle_ms < BARCODE_IDLE_TIMEOUT_MS) {
        // publish duty snapshot while mid-pass
        float b,w,d; uint32_t age;
        barcode_get_duty_ms(&b,&w,&d,&age);
        mqtt_publish_barcode_duty(b,w,d);
        return false;
    }

    // Close trailing open segment
    if (bcap.last_level >= 0 && bcap.n < BARCODE_MAX_ELEMS) {
        uint32_t dt = (uint32_t)(now_us - bcap.t_last);
        bcap.dur_us[bcap.n] = dt;
        bcap.lvl[bcap.n]    = (uint8_t)bcap.last_level;
        bcap.n++;
        if (bcap.last_level) bcap.acc_black_us += dt; else bcap.acc_white_us += dt;
        mqtt_publish_barcode_edge(bcap.last_level, dt);
    }

    // Duty snapshot
    float b,w,d; uint32_t age;
    barcode_get_duty_ms(&b,&w,&d,&age);
    mqtt_publish_barcode_duty(b,w,d);

    if (bcap.n < 9) { barcode_reset(); return false; }

    // Threshold to narrow/wide
    float thr = choose_threshold_us(bcap.dur_us, bcap.n);
    if (thr <= 0.f) { barcode_reset(); return false; }

    uint8_t nw[BARCODE_MAX_ELEMS];
    for (int i=0;i<bcap.n;i++) nw[i] = (bcap.dur_us[i] > thr) ? 1 : 0;

    // Skip leading spaces
    int i = 0; while (i < bcap.n && bcap.lvl[i]==0) i++;

    char buf[64]; int blen=0;
    int guards_seen=0;
    int chars_total=0;

    while (i + 9 <= bcap.n) {
        // Must start with BAR and alternate
        bool ok=true;
        for (int k=0;k<9;k++){
            int expected = (k%2==0)?1:0;
            if (bcap.lvl[i+k] != expected) { ok=false; break; }
        }
        if (!ok) { i++; continue; }

        unsigned short pat = pack9(&nw[i]);
        int idx = c39_find(pat);
        if (idx < 0) { i++; continue; }

        char ch = C39_MAP[idx].ch;
        if (ch == '*') {
            guards_seen++;
        } else if (blen < (int)sizeof(buf)-1) {
            buf[blen++] = ch;
        }

        chars_total++;

        // Consume inter-character narrow space if present
        int advance = 9;
        if (i + 9 < bcap.n && bcap.lvl[i+9]==0) advance = 10;
        i += advance;

        if (guards_seen >= 2 && (i >= bcap.n - 2)) break;
    }

    bool success = (guards_seen >= 2 && blen > 0);
    float conf = success ? fminf(1.f, (float)blen/(float)(chars_total>0?chars_total:1)) : 0.f;

    if (success && out && out_sz>0){
        int copy = (blen < out_sz-1) ? blen : (out_sz-1);
        memcpy(out, buf, copy); out[copy]='\0';
        if (conf_out) *conf_out = conf;
    }

    barcode_reset();
    return success;
}

// ===================== Globals for LF =====================
static absolute_time_t next_print;
static uint16_t threshold = 2100;    // tune to your tape/lighting
static float    ir_raw_ema = 0.0f;

// ===================== Setup/Init =====================
static void wifi_connect_or_exit(void) {
    if (cyw43_arch_init()) {
        printf("❌ CYW43 init failed\n");
        while(1) tight_loop_contents();
    }
    cyw43_arch_enable_sta_mode();
    printf("Wi-Fi: connecting…\n");
}

static void gpio_init_all(void) {
    // ADC for LF
    adc_init();
    adc_gpio_init(LF_ADC_GPIO);
    adc_select_input(0); // ADC0 for GP26

    // Barcode D0 input (use pull-up, most LM393 are active-low on black)
    gpio_init(BAR_D0_GPIO);
    gpio_set_dir(BAR_D0_GPIO, false);
    gpio_pull_up(BAR_D0_GPIO);

    // Encoders + motors
    encoder_init_pair(ENC_LEFT_GPIO, ENC_RIGHT_GPIO);   // <-- fixed signature
    motor_init(M1A, M1B, M2A, M2B);

    // IMU, if present
    imu_init();
}

// ===================== Main =====================
int main(void) {
    stdio_init_all();
    sleep_ms(200);

    wifi_connect_or_exit();
    mqtt_client_start("192.168.1.6", 1883);   // adjust to your broker
    next_print = make_timeout_time_ms(PRINT_MS);

    gpio_init_all();

    // Barcode edge sampler state
    int prev_bar_level = -1;

    // LF control state
    int32_t lf_white_ms = 0;
    float u_filt = 0.0f;
    float left_pct = 0.0f, right_pct = 0.0f;
    float base_pct_nominal = LF_BASE_SPEED_PCT;

    while (true) {
        // ---- Read analog LF
        adc_select_input(0);
        uint16_t ir_raw = adc_read();
        if (ir_raw_ema == 0.0f) ir_raw_ema = (float)ir_raw;
        ir_raw_ema = (1.0f - IR_EMA_ALPHA) * ir_raw_ema + IR_EMA_ALPHA * (float)ir_raw;

        // Decide “on black?” from analog threshold (no LF D0 needed)
        bool on_black = ((int32_t)ir_raw_ema >= (int32_t)threshold);
        if (!on_black) lf_white_ms += SAMPLE_IR_MS; else lf_white_ms = 0;

        // ---- Barcode edge sampling (level normalize to 1=black, 0=white)
        int raw = gpio_get(BAR_D0_GPIO);
        int bar_level = (raw == 0) ? 1 : 0;   // active-low boards: 0→black => map to 1
        uint64_t now_us = time_us_64();

        if (prev_bar_level < 0) prev_bar_level = bar_level;
        if (bar_level != prev_bar_level) {
            barcode_on_edge(bar_level, now_us);
            prev_bar_level = bar_level;
        }

        // ---- Line following (ADC-based PID)
#if LF_ENABLE
        // Error relative to threshold
        int32_t err = (int32_t)ir_raw_ema - (int32_t)threshold;
        if (err > -LF_ERR_DEADBAND && err < LF_ERR_DEADBAND) err = 0;

        static int32_t prev_err = 0;
        int32_t derr = err - prev_err; prev_err = err;

        float u = LF_KP * (float)err + LF_KD * (float)derr;
        u_filt = (1.0f - LF_U_SMOOTH_ALPHA) * u_filt + LF_U_SMOOTH_ALPHA * u;

        // Base speed (slow down a bit while capturing barcode)
        float base_pct = base_pct_nominal;
        if (barcode_is_capturing()) base_pct = fminf(base_pct, 24.0f);

        float turn = fmaxf(-LF_TURN_PCT, fminf(LF_TURN_PCT, u_filt));

        float left_target  = base_pct - turn;
        float right_target = base_pct + turn;

        // Slew limit
        float dl = fmaxf(-SLEW_PCT_PER_TICK, fminf(SLEW_PCT_PER_TICK, left_target  - left_pct));
        float dr = fmaxf(-SLEW_PCT_PER_TICK, fminf(SLEW_PCT_PER_TICK, right_target - right_pct));
        left_pct  += dl;
        right_pct += dr;

        // Minimum wheel floor if moving
        if (fabsf(left_pct)  > 0 && fabsf(left_pct)  < MIN_WHEEL_PCT)  left_pct  = copysignf(MIN_WHEEL_PCT,  left_pct);
        if (fabsf(right_pct) > 0 && fabsf(right_pct) < MIN_WHEEL_PCT) right_pct  = copysignf(MIN_WHEEL_PCT, right_pct);

        // Lost-line recovery / reverse assist
        if (lf_white_ms >= LF_LOST_MS) {
#if WHITE_REV_ENABLE
            // brief reverse with bias to last turn direction
            float rev_l = -WHITE_REV_PCT + (u_filt < 0 ? -WHITE_REV_TURN_PCT : +WHITE_REV_TURN_PCT);
            float rev_r = -WHITE_REV_PCT + (u_filt < 0 ? +WHITE_REV_TURN_PCT : -WHITE_REV_TURN_PCT);
            motor_set_speed(rev_l, rev_r);                 // <-- API fix
            sleep_ms(WHITE_REV_MIN_MS);
#else
            // spin to reacquire
            float spin = (u_filt < 0) ? -LF_RECOVER_PCT : +LF_RECOVER_PCT;
            motor_set_speed(-spin, +spin);                 // <-- API fix
            sleep_ms(80);
#endif
            lf_white_ms = 0;
        } else {
            motor_set_speed(left_pct, right_pct);          // <-- API fix
        }
#endif // LF_ENABLE

        // ---- Finalize barcode pass (idle window)
        char decoded[32]; float conf=0.f;
        if (barcode_finalize_if_ready(now_us, decoded, sizeof(decoded), &conf)) {
            const char *guess = "UNKNOWN";
            if      (strcmp(decoded, "A") == 0) guess = "RIGHT";
            else if (strcmp(decoded, "Z") == 0) guess = "LEFT";

            printf("[BAR39] \"%s\" conf=%.2f -> %s\n", decoded, conf, guess);
            // Keep existing MQTT schema but publish text under snapshot topic
            mqtt_publish_barcode_snapshot(decoded, (int)strlen(decoded), guess, conf);
        }

        // ---- Encoder / IMU / MQTT publish cadence
        absolute_time_t now = get_absolute_time();
        bool do_print = absolute_time_diff_us(now, next_print) <= 0;

        // keep lwIP timers moving every loop (prevents burstiness)
        mqtt_loop_poll();

        if (do_print) {
            mqtt_publish_imu();
            mqtt_publish_speed();
            mqtt_publish_distance();

            // line telemetry via generic publisher
            char payload[128];
            snprintf(payload, sizeof(payload),
                "{\"ir_raw\":%u,\"ir_ema\":%.1f,\"thr\":%u}",
                (unsigned)ir_raw, ir_raw_ema, (unsigned)threshold);
            mqtt_publish_message("telemetry/line", payload);   // <-- use generic

            next_print = make_timeout_time_ms(PRINT_MS);
        }

        sleep_ms(SAMPLE_IR_MS);
    }
}
