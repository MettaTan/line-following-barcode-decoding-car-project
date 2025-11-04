#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "encoder.h"   // needs enc_id_t { ENC_LEFT, ENC_RIGHT }, encoder_init_pair, encoder_get_count(id), encoder_get_period_us(id)

// =================== Pin map ===================
#define IR_ADC_GPIO     26      // GP26 -> ADC0 (A0)
#define IR_D0_GPIO      27      // GP27 -> digital comparator out (D0)
#define IR_ADC_INPUT     0      // ADC0

#define ENC_LEFT_GPIO     2     // We wire single encoder OUT to GP2
#define ENC_RIGHT_GPIO  0xFFFFFFFFu // Unused (no second encoder)

// Optional PWM “visual” stub (not motor drive)
#define PWM_GPIO         0      // GP0
#define DIR1_GPIO        1
#define DIR2_GPIO        3

// =================== Timings ===================
#define SAMPLE_IR_MS    25
#define PRINT_MS       250
#define CALIB_MS       800

#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV   20
#endif

// ============ Barcode capture/decoding =========
// How long with no edges to consider the pass "ended"
#define BARCODE_IDLE_TIMEOUT_MS   400
// Max bars to capture per pass (black segments)
#define MAX_BARS                  32
// When classifying thick vs thin, adaptive threshold between min and max
// If distribution is tight, fall back to ratio vs median.
#define MIN_SEPARATION_RATIO      1.35f   // if (max/min) < this, widths are too similar

// Editable reference patterns (thin=0, thick=1). Adjust after your first real scan:
#define A_PATTERN "001101000100110"
#define Z_PATTERN "001100110000110"

// ============ PWM helper (visual only) =========
static void pwm_init_stub(void) {
    gpio_set_function(PWM_GPIO, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_GPIO);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 65535);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(PWM_GPIO, 0);

    gpio_init(DIR1_GPIO); gpio_set_dir(DIR1_GPIO, true); gpio_put(DIR1_GPIO, 1);
    gpio_init(DIR2_GPIO); gpio_set_dir(DIR2_GPIO, true); gpio_put(DIR2_GPIO, 0);
}

static inline void pwm_set_duty_percent(uint8_t pct) {
    if (pct > 100) pct = 100;
    uint16_t level = (uint16_t)((65535u * pct) / 100u);
    pwm_set_gpio_level(PWM_GPIO, level);
}

// ============ ADC quick read ============
static inline uint16_t adc_read_raw(void) {
    adc_select_input(IR_ADC_INPUT);
    return adc_read(); // 0..4095
}

// ============ D0 windowed duty tracker ============
static uint64_t d0_last_edge_us = 0;
static int      d0_prev         = -1;
static uint32_t acc_black_us    = 0;
static uint32_t acc_white_us    = 0;
static uint32_t d0_age_ms       = 0;

static inline void d0_duty_reset(void) {
    acc_black_us = acc_white_us = 0;
    d0_age_ms = 0;
    d0_last_edge_us = time_us_64();
}

static inline float d0_black_duty(float *black_ms, float *white_ms) {
    uint32_t total = acc_black_us + acc_white_us;
    if (total == 0) {
        if (black_ms) *black_ms = 0;
        if (white_ms) *white_ms = 0;
        return 0.f;
    }
    if (black_ms) *black_ms = acc_black_us / 1000.f;
    if (white_ms) *white_ms = acc_white_us / 1000.f;
    return 100.f * ((float)acc_black_us / (float)total);
}

// ============ Barcode width capture on D0 ============
// We collect widths of HIGH segments (black bars)
static uint32_t black_width_us[MAX_BARS];
static int      black_count = 0;
static bool     capturing   = false;
static uint64_t pass_last_edge_us = 0;   // last edge time (for idle timeout)
static uint64_t black_start_us = 0;      // start time of current black pulse

static void barcode_capture_reset(void) {
    black_count = 0;
    capturing = false;
    pass_last_edge_us = time_us_64();
    black_start_us = 0;
}

static void barcode_on_edge(int prev_level, int now_level, uint64_t now_us) {
    // Track accumulators for duty
    uint32_t dt = (uint32_t)(now_us - d0_last_edge_us);
    if (prev_level == 1) acc_black_us += dt; else if (prev_level == 0) acc_white_us += dt;
    d0_last_edge_us = now_us;
    d0_age_ms = 0;

    // Start pass when we first see BLACK (white->black)
    if (!capturing && prev_level == 0 && now_level == 1) {
        capturing = true;
        black_count = 0;
        black_start_us = now_us;
    } else if (capturing) {
        // If we are capturing, record BLACK pulse width at black->white edge
        if (prev_level == 1 && now_level == 0) {
            if (black_start_us > 0 && black_count < MAX_BARS) {
                uint32_t width = (uint32_t)(now_us - black_start_us);
                black_width_us[black_count++] = width;
            }
        } else if (prev_level == 0 && now_level == 1) {
            // new black just started
            black_start_us = now_us;
        }
    }
    pass_last_edge_us = now_us;
}

// Adaptive thresholding & decoding
static int classify_widths_to_bits(const uint32_t *w, int n, char *out_bits, int out_sz) {
    if (n <= 0 || out_sz < (n + 1)) return -1;

    // Find min, max, median-ish
    uint32_t minw = 0xFFFFFFFFu, maxw = 0;
    uint64_t sum = 0;
    for (int i = 0; i < n; ++i) {
        if (w[i] < minw) minw = w[i];
        if (w[i] > maxw) maxw = w[i];
        sum += w[i];
    }
    float avg = (float)sum / (float)n;

    // Threshold: if clear separation, use midpoint of min/max; otherwise bias above average
    float thr;
    if (minw > 0 && ((float)maxw / (float)minw) >= MIN_SEPARATION_RATIO) {
        thr = 0.5f * ((float)minw + (float)maxw);
    } else {
        thr = avg * 1.20f; // push above average so "thick" must be meaningfully larger
    }

    for (int i = 0; i < n; ++i) {
        out_bits[i] = (w[i] > thr) ? '1' : '0';
    }
    out_bits[n] = '\0';
    return n;
}

static const char* match_pattern(const char *bits) {
    // Compare against your editable A/Z patterns; update them after measuring your sheets once.
    if (strcmp(bits, A_PATTERN) == 0) return "A (TURN RIGHT)";
    if (strcmp(bits, Z_PATTERN) == 0) return "Z (TURN LEFT)";
    return "UNKNOWN";
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    // ---- IR (ADC + D0) ----
    adc_init();
    adc_gpio_init(IR_ADC_GPIO);
    adc_select_input(IR_ADC_INPUT);
    gpio_init(IR_D0_GPIO);
    gpio_set_dir(IR_D0_GPIO, false);
    gpio_pull_down(IR_D0_GPIO);

    // ---- Encoder (single on LEFT=GP2) ----
    encoder_init_pair(ENC_LEFT_GPIO, ENC_RIGHT_GPIO);

    // ---- PWM visual stub ----
    pwm_init_stub();

    // ---- Auto-calibrate IR threshold ----
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint16_t minv = 0xFFFF, maxv = 0;
    while (to_ms_since_boot(get_absolute_time()) - t0 < CALIB_MS) {
        uint16_t v = adc_read_raw();
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
        sleep_ms(10);
    }
    uint16_t threshold = (uint16_t)((minv + maxv) / 2);
    if ((maxv - minv) < 300) threshold = 2300; // fallback based on your lab observations
    printf("[CAL] min=%u max=%u thr=%u\n", minv, maxv, threshold);

    // ---- Loop timers ----
    absolute_time_t next_sample = make_timeout_time_ms(SAMPLE_IR_MS);
    absolute_time_t next_print  = make_timeout_time_ms(PRINT_MS);

    // ---- Init trackers ----
    d0_prev = gpio_get(IR_D0_GPIO);
    d0_last_edge_us = time_us_64();
    d0_duty_reset();
    barcode_capture_reset();

    // Simple virtual P-visualization
    float L_pct = 50.f, R_pct = 50.f;

    // ---- Main loop ----
    while (true) {
        // Periodic sample
        if (absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {
            // Analog IR
            uint16_t ir_raw = adc_read_raw();
            bool line_black = (ir_raw >= threshold);

            // Digital D0, edge tracking
            int d0_now = gpio_get(IR_D0_GPIO);
            uint64_t now_us = time_us_64();
            if (d0_now != d0_prev) {
                barcode_on_edge(d0_prev, d0_now, now_us);
                d0_prev = d0_now;
            } else {
                d0_age_ms += SAMPLE_IR_MS;
            }

            // Finish currently active segment for running duty (not for barcode capture)
            // (we add to accumulators only on edges to avoid double counting here)

            // Tiny P-like visualization to PWM (not actual motor drive)
            float span = (float)(maxv > minv ? (maxv - minv) : 1);
            float err = ((float)ir_raw - (float)threshold) / (span * 0.5f);
            if (err < -1.f) err = -1.f; if (err > 1.f) err = 1.f;
            const float KP = 20.f;
            float steer = KP * err; // + -> favor R
            L_pct = 50.f - steer; R_pct = 50.f + steer;
            if (L_pct < 0) L_pct = 0; if (L_pct > 100) L_pct = 100;
            if (R_pct < 0) R_pct = 0; if (R_pct > 100) R_pct = 100;
            pwm_set_duty_percent((uint8_t)R_pct);

            // If we were capturing and go idle for a while, treat as end-of-pass
            if (capturing) {
                uint64_t now_ms = to_ms_since_boot(get_absolute_time());
                (void)now_ms;
                if (((time_us_64() - pass_last_edge_us) / 1000u) > BARCODE_IDLE_TIMEOUT_MS) {
                    // End-of-pass: classify what we collected
                    char bits[MAX_BARS + 1] = {0};
                    int nbits = classify_widths_to_bits(black_width_us, black_count, bits, sizeof(bits));
                    if (nbits > 0) {
                        const char *which = match_pattern(bits);
                        printf("[BAR] count=%d bits=%s → %s\n", nbits, bits, which);
                    } else {
                        printf("[BAR] no bits captured\n");
                    }
                    barcode_capture_reset();
                }
            }

            next_sample = delayed_by_ms(next_sample, SAMPLE_IR_MS);
        }

        // Periodic print
        if (absolute_time_diff_us(get_absolute_time(), next_print) <= 0) {
            // Close current D0 segment into duty window for this print
            uint64_t now_us = time_us_64();
            uint32_t seg = (uint32_t)(now_us - d0_last_edge_us);
            if (d0_prev == 1) acc_black_us += seg; else if (d0_prev == 0) acc_white_us += seg;
            d0_last_edge_us = now_us;

            float black_ms = 0.f, white_ms = 0.f;
            float duty_black = d0_black_duty(&black_ms, &white_ms);

            // Read analog once for display
            uint16_t ir_raw = adc_read_raw();
            const char *state = (ir_raw >= threshold) ? "BLACK" : "WHITE";
            int d0_last = gpio_get(IR_D0_GPIO);

            // Encoders (only LEFT is wired on GP2)
            uint32_t ticks_L   = encoder_get_count(ENC_LEFT);
            uint32_t per_us_L  = encoder_get_period_us(ENC_LEFT);
            float rps_L = per_us_L ? (1e6f / (float)per_us_L) / (float)SLOTS_PER_REV : 0.f;
            float rpm_L = rps_L * 60.f;

            // Echo any captured widths so far (debug)
            printf("raw=%u thr=%u state=%s d0=%d "
                   "black_ms=%.1f white_ms=%.1f black_duty=%.1f%% age_ms=%u "
                   "L=%.0f%% R=%.0f%% L_ticks=%lu L_rpm=%.2f "
                   "bars=%d\n",
                   ir_raw, threshold, state, d0_last,
                   black_ms, white_ms, duty_black, d0_age_ms,
                   L_pct, R_pct,
                   (unsigned long)ticks_L, rpm_L,
                   black_count);

            // Reset duty window after print
            d0_duty_reset();

            next_print = delayed_by_ms(next_print, PRINT_MS);
        }

        tight_loop_contents();
    }
}
