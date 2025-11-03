#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include "encoder.h"   // must provide: enc_id_t { ENC_LEFT, ENC_RIGHT }, encoder_init_pair, encoder_get_count(id), encoder_get_period_us(id)

// =================== Pin map (your wiring) ===================
#define IR_ADC_GPIO   26      // GP26 -> ADC0 (A0)
#define IR_D0_GPIO    27      // GP27 -> digital comparator output (D0)
#define IR_ADC_INPUT   0      // ADC0

#define ENC_LEFT_GPIO   2     // wheel encoder LEFT on GP2  (GROVE #2 SIG1)
#define ENC_RIGHT_GPIO  9     // wheel encoder RIGHT on GP9 (GROVE #5 ??? adjust if different)

// PWM stub (for on-screen “steering” only; not driving real motors)
#define PWM_GPIO        0     // GP0 for a visible PWM pin (scope/LED), optional
#define DIR1_GPIO       1
#define DIR2_GPIO       3

// =================== Timings / constants =====================
#define SAMPLE_IR_MS   25
#define PRINT_MS      250
#define CALIB_MS      800

#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV 20
#endif

// -------------------------------------------------------------
// Small PWM helper (optional visual feedback)
static void pwm_init_stub(void) {
    gpio_set_function(PWM_GPIO, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(PWM_GPIO);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_wrap(&cfg, 65535);          // ~1.9 kHz at 125 MHz
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

// Quick ADC read (12-bit 0..4095)
static inline uint16_t adc_read_raw(void) {
    adc_select_input(IR_ADC_INPUT);
    return adc_read();
}

// -------------------- D0 pulse “duty” tracker ----------------
// Measures how long D0 stays HIGH (black) vs LOW (white) since last flip.
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

// Returns current duty% as (black time / total) * 100
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

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    // ---------- IR (ADC + D0) ----------
    adc_init();
    adc_gpio_init(IR_ADC_GPIO);
    adc_select_input(IR_ADC_INPUT);

    gpio_init(IR_D0_GPIO);
    gpio_set_dir(IR_D0_GPIO, false);   // input
    gpio_pull_down(IR_D0_GPIO);        // module drives actively; pull helps on float

    // ---------- Encoders (LEFT + RIGHT) ----------
    // Your encoder.h must expose this function for a pair init.
    encoder_init_pair(ENC_LEFT_GPIO, ENC_RIGHT_GPIO);

    // ---------- PWM visual stub ----------
    pwm_init_stub();

    // ---------- IR auto-calibrate ----------
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint16_t minv = 0xFFFF, maxv = 0;
    while (to_ms_since_boot(get_absolute_time()) - t0 < CALIB_MS) {
        uint16_t v = adc_read_raw();
        if (v < minv) minv = v;
        if (v > maxv) maxv = v;
        sleep_ms(10);
    }
    uint16_t threshold = (uint16_t)((minv + maxv) / 2);
    // If contrast was poor, pick a sane default (you had ~200 white / ~3700 black)
    if ((maxv - minv) < 300) threshold = 2300;
    printf("[CAL] min=%u max=%u thr=%u\n", minv, maxv, threshold);

    // ---------- Loop timers ----------
    absolute_time_t next_sample = make_timeout_time_ms(SAMPLE_IR_MS);
    absolute_time_t next_print  = make_timeout_time_ms(PRINT_MS);

    // D0 duty tracking init
    d0_prev = gpio_get(IR_D0_GPIO);
    d0_last_edge_us = time_us_64();
    d0_duty_reset();

    // P-steer “virtual” outputs (not actually driving motors)
    float L_pct = 50.f, R_pct = 50.f;

    // ---------- Main loop ----------
    while (true) {
        // Sample block (every SAMPLE_IR_MS)
        if (absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {

            // --- Analog reflectance ---
            uint16_t ir_raw = adc_read_raw();
            bool line_black = (ir_raw >= threshold);

            // --- Digital D0 measure (accumulate time between edges) ---
            int d0_now = gpio_get(IR_D0_GPIO);
            uint64_t now_us = time_us_64();
            if (d0_now != d0_prev) {
                uint32_t dt = (uint32_t)(now_us - d0_last_edge_us);
                if (d0_prev == 1) acc_black_us += dt; else if (d0_prev == 0) acc_white_us += dt;
                d0_last_edge_us = now_us;
                d0_prev = d0_now;
                d0_age_ms = 0;               // “fresh” measurement
            } else {
                // add running time to the current state so duty reflects continuous time
                uint32_t dt = (uint32_t)(now_us - d0_last_edge_us);
                // do not push into acc_* here to avoid double counting; just age
                d0_age_ms += SAMPLE_IR_MS;
            }

            // --- Very small P-steer (virtual) from analog error ---
            // Map raw (minv..maxv) -> error in [-1..+1] around threshold
            float err = 0.f;
            if (maxv > minv) {
                float span = (float)(maxv - minv);
                float center = (float)threshold;
                err = ((float)ir_raw - center) / (span * 0.5f);   // about ±1 near extremes
                if (err < -1.f) err = -1.f;
                if (err > +1.f) err = +1.f;
            }
            // P gain small; just to show changing “L/R”
            const float KP = 20.f;   // % per unit error
            float steer = KP * err;  // positive -> favor RIGHT
            L_pct = 50.f - steer;
            R_pct = 50.f + steer;
            if (L_pct < 0) L_pct = 0; if (L_pct > 100) L_pct = 100;
            if (R_pct < 0) R_pct = 0; if (R_pct > 100) R_pct = 100;

            // Optional PWM pin to visualize change
            pwm_set_duty_percent((uint8_t)R_pct);

            next_sample = delayed_by_ms(next_sample, SAMPLE_IR_MS);
        }

        // Print block (every PRINT_MS)
        if (absolute_time_diff_us(get_absolute_time(), next_print) <= 0) {

            // Finish the currently active D0 segment into its bucket so numbers move
            uint64_t now_us = time_us_64();
            uint32_t seg = (uint32_t)(now_us - d0_last_edge_us);
            if (d0_prev == 1) acc_black_us += seg; else if (d0_prev == 0) acc_white_us += seg;
            d0_last_edge_us = now_us;

            float black_ms = 0.f, white_ms = 0.f;
            float duty_black = d0_black_duty(&black_ms, &white_ms);

            // Read analog once for the print line and compute state
            uint16_t ir_raw = adc_read_raw();
            const char *state = (ir_raw >= threshold) ? "BLACK" : "WHITE";
            int d0_last = gpio_get(IR_D0_GPIO);

            // Encoders: both wheels
            uint32_t ticks_L   = encoder_get_count(ENC_LEFT);
            uint32_t ticks_R   = encoder_get_count(ENC_RIGHT);
            uint32_t per_us_L  = encoder_get_period_us(ENC_LEFT);
            uint32_t per_us_R  = encoder_get_period_us(ENC_RIGHT);
            float rps_L = per_us_L ? (1e6f / (float)per_us_L) / (float)SLOTS_PER_REV : 0.f;
            float rps_R = per_us_R ? (1e6f / (float)per_us_R) / (float)SLOTS_PER_REV : 0.f;
            float rpm_L = rps_L * 60.f;
            float rpm_R = rps_R * 60.f;

            printf("raw=%u thr=%u state=%s d0=%d "
                   "black_us=%.0fms white_us=%.0fms black_duty=%.1f%% age_ms=%u "
                   "L=%.0f%% R=%.0f%% "
                   "L_ticks=%lu R_ticks=%lu L_rpm=%.2f R_rpm=%.2f\n",
                   ir_raw, threshold, state, d0_last,
                   black_ms, white_ms, duty_black, d0_age_ms,
                   L_pct, R_pct,
                   (unsigned long)ticks_L, (unsigned long)ticks_R, rpm_L, rpm_R);

            // After printing, don’t double count this segment on the next pass
            // (we reset the accumulators to start a fresh window)
            d0_duty_reset();

            next_print = delayed_by_ms(next_print, PRINT_MS);
        }

        tight_loop_contents();
    }
}
