#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "encoder.h"
#include "motor.h"
#include "imu.h"
#include "line.h"

// -------------------- Pin map (MUST MATCH WIRING) --------------------
#define IR_ADC_GPIO      26      // A0 -> GP26/ADC0  (IR1 analog used for LF + barcode)
#define IR_D0_GPIO       27      // D0 -> GP27 (digital comparator output)

// Second IR (line follower) D0
#define IR2_D0_GPIO 13

#define ENC_LEFT_GPIO     2      // Left wheel encoder OUT -> GP2
#define ENC_RIGHT_GPIO    4      // Right wheel encoder OUT -> GP4

// Keep your motor pins / wiring (used by motor.c)
#define M1A               8
#define M1B               9
#define M2A              10
#define M2B              11

// -------------------- Timings / params --------------------
#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV     20
#endif

#define SAMPLE_IR_MS      2      // fast loop helps both barcode and LF
#define PRINT_MS         250
#define CALIB_MS         800

// --------- DRIVE / LINE-FOLLOW (IR2 D0 on GP13) ---------
#define LF_ENABLE             1        // 1 = enable line-follow (uses IR2 D0 only)
#define LF_BASE_SPEED_PCT    36.0f     // forward base speed
#define LF_TURN_PCT          34.0f     // steering authority (percent of full scale)
#define LF_KP                 0.38f     // PD on ±1 digital error
#define LF_KD                 0.08f     // small derivative damping (single source!)

#define LF_D0_MIN_HOLD_MS      4       // debounce/hold for D0 flips (react faster on curves)
#define SLEW_PCT_PER_TICK      7.0f    // wheel % change cap per control tick (limits twitch)
#define LF_MIN_WHEEL_PCT      22.0f    // torque floor so wheels don’t stall on turns
#define LF_INVERT_STEER        1       // 0/1: flip if it turns the wrong way

#define LF_FLIP_CONFIRM_MS    70    // require this long before accepting steer sign flip
#define CURVE_ANALOG_EPS    120.0f  // keep (used below if you left assist on)

// Optional micro-assist from IR1 analog (helps on tight curves). Set to 0.0f to disable.
#define CURVE_ASSIST_ALPHA     0.20f   // blend weight for analog assist (0..0.3 typical)
#define CURVE_ANALOG_EPS     120.0f    // normalisation for analog error (~threshold band)

// Lost-line behaviour (only used if you added a WHITE timer)
#define LF_LOST_MS            450      // ms on WHITE before recovery behaviour
#define LF_RECOVER_PCT         22.0f   // slow spin speed during recovery

// --------- Encoder drift trim (optional) ---------
#define DRIFT_PID_ENABLE       1
#define DRIFT_KP               0.003f
#define DRIFT_KI               0.00001f
#define DRIFT_KD               0.0005f
#define DRIFT_CORR_MAX        20.0f    // limit drift trim to ±20% speed

// --------- Barcode capture ---------
#define BARCODE_IDLE_TIMEOUT_MS 1500   // allow longer gaps between edges
#define MAX_BARS                 64    // headroom for thin/small codes
#define MIN_SEPARATION_RATIO    1.35f  // width separation heuristic for 0/1 classification

// Editable reference patterns (your sheets)
#define A_PATTERN "001101000100110"
#define Z_PATTERN "001100110000110"


static uint32_t lf_white_ms = 0;

static float ir_raw_ema = 0.0f;
static float u_filt = 0.0f;
static float left_cmd_pct = 0.0f, right_cmd_pct = 0.0f;   // for slewing
static int32_t prev_err = 0;

// -------------------- D0 duty/width trackers --------------------
static uint64_t d0_last_edge_us = 0;
static int      d0_prev         = -1;
static uint32_t acc_black_us    = 0;
static uint32_t acc_white_us    = 0;
static uint32_t d0_age_ms       = 0;

static int      analog_state    = 0;   // 0=WHITE, 1=BLACK (for Schmitt)
static uint16_t thr_low         = 0, thr_high = 0;

static int      lf_prev_d0 = -1;
static uint32_t lf_state_hold_ms = 0;

static inline void d0_duty_reset(void) {
    acc_black_us = acc_white_us = 0;
    d0_age_ms = 0;
    d0_last_edge_us = time_us_64();
}

static inline float d0_black_duty(float *black_ms, float *white_ms) {
    uint32_t total = acc_black_us + acc_white_us;
    if (total == 0) { if (black_ms) *black_ms = 0; if (white_ms) *white_ms = 0; return 0.f; }
    if (black_ms) *black_ms = acc_black_us / 1000.f;
    if (white_ms) *white_ms = acc_white_us / 1000.f;
    return 100.f * ((float)acc_black_us / (float)total);
}

// -------------------- Barcode state (D0 HIGH = black) --------------------
static uint32_t black_width_us[MAX_BARS];
static int      black_count = 0;
static bool     capturing   = false;
static uint64_t pass_last_edge_us = 0;
static uint64_t black_start_us    = 0;

static int8_t   lf_cmd_sign = 0;      // current accepted steer sign (-1,0,+1)
static uint32_t lf_flip_ms  = 0;      // how long a new sign has been requested

static void barcode_capture_reset(void) {
    black_count = 0; capturing = false;
    pass_last_edge_us = time_us_64();
    black_start_us = 0;
}

static void barcode_on_edge(int prev_level, int now_level, uint64_t now_us) {
    // duty accumulation
    uint32_t dt = (uint32_t)(now_us - d0_last_edge_us);
    if (prev_level == 1) acc_black_us += dt; else if (prev_level == 0) acc_white_us += dt;
    d0_last_edge_us = now_us; d0_age_ms = 0;

    // Start capture on the first edge we ever see
    if (!capturing) { capturing = true; black_count = 0; black_start_us = 0; }

    // Record/advance as edges arrive
    if (prev_level == 1 && now_level == 0) {            // BLACK -> WHITE closes a black bar
        if (black_start_us > 0 && black_count < MAX_BARS) {
            uint32_t width = (uint32_t)(now_us - black_start_us);
            black_width_us[black_count++] = width;
        }
    } else if (prev_level == 0 && now_level == 1) {     // WHITE -> BLACK starts a black bar
        black_start_us = now_us;
    }
    pass_last_edge_us = now_us;
}

static int classify_widths_to_bits(const uint32_t *w, int n, char *out_bits, int out_sz) {
    if (n <= 0 || out_sz < (n + 1)) return -1;
    uint32_t minw = 0xFFFFFFFFu, maxw = 0; uint64_t sum = 0;
    for (int i = 0; i < n; ++i) { if (w[i] < minw) minw = w[i]; if (w[i] > maxw) maxw = w[i]; sum += w[i]; }
    float avg = (float)sum / (float)n;
    float thr = (minw > 0 && ((float)maxw / (float)minw) >= MIN_SEPARATION_RATIO)
              ? 0.5f * ((float)minw + (float)maxw)
              : avg * 1.20f;
    for (int i = 0; i < n; ++i) out_bits[i] = (w[i] > thr) ? '1' : '0';
    out_bits[n] = '\0';
    return n;
}

static const char* match_pattern(const char *bits) {
    if (strcmp(bits, A_PATTERN) == 0) return "A (RIGHT)";
    if (strcmp(bits, Z_PATTERN) == 0) return "Z (LEFT)";
    return "UNKNOWN";
}

// -------------------- Drift PID (encoders) --------------------
typedef struct { float kp, ki, kd, integ, prev, out_min, out_max; } PID;
static inline void pid_init(PID* p, float kp, float ki, float kd, float mn, float mx){ p->kp=kp;p->ki=ki;p->kd=kd;p->integ=0;p->prev=0;p->out_min=mn;p->out_max=mx; }
static inline float pid_update(PID* p, float setpoint, float current){
    float e = setpoint - current; p->integ += e; float d = e - p->prev; p->prev = e;
    float u = p->kp*e + p->ki*p->integ + p->kd*d;
    if (u > p->out_max) u = p->out_max; else if (u < p->out_min) u = p->out_min;
    return u;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1200);

    printf("\n=== Week10 Integration: IR + Encoders + Barcode + (LF add-on) ===\n");

    // ---- IR (analog via line.c + digital D0) ----
    irsens_init(IR_ADC_GPIO);
    gpio_init(IR_D0_GPIO); gpio_set_dir(IR_D0_GPIO, false); gpio_pull_down(IR_D0_GPIO);

    gpio_init(IR2_D0_GPIO); gpio_set_dir(IR2_D0_GPIO, false); gpio_pull_down(IR2_D0_GPIO);

    // ---- Encoders ----
    encoder_init_pair(ENC_LEFT_GPIO, ENC_RIGHT_GPIO);

    // ---- Motors ----
    motor_init(M1A, M1B, M2A, M2B);
    motor_stop();

    // ---- IMU (preserve structure) ----
    imu_init();

    // ---- Auto-calibrate IR threshold from analog A0 (robust) ----
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint16_t minv = 0xFFFF, maxv = 0;
    while (to_ms_since_boot(get_absolute_time()) - t0 < CALIB_MS) {
        int ar = irsens_read_analog(); uint16_t v = (ar < 0) ? 0 : (uint16_t)ar;
        if (v < minv) minv = v; if (v > maxv) maxv = v; sleep_ms(5);
    }
    uint16_t span = (maxv > minv) ? (maxv - minv) : 0;
    uint16_t threshold = (span >= 60) ? (uint16_t)(minv + span / 2) : 600;
    printf("[CAL] min=%u max=%u span=%u thr=%u\n", minv, maxv, span, threshold);

    // Schmitt band ~10% of threshold, min 60 counts
    uint16_t band = (uint16_t)(threshold / 10); if (band < 60) band = 60;
    thr_low  = (threshold > band) ? (threshold - band) : (threshold / 2);
    thr_high = threshold + band;

    // Seed from first sample
    int ar0 = irsens_read_analog(); analog_state = (ar0 >= threshold) ? 1 : 0;

    // ---- Loop timers ----
    absolute_time_t next_sample = make_timeout_time_ms(SAMPLE_IR_MS);
    absolute_time_t next_print  = make_timeout_time_ms(PRINT_MS);

    // ---- Init edge/duty trackers ----
    d0_prev = gpio_get(IR_D0_GPIO);
    d0_last_edge_us = time_us_64();
    d0_duty_reset();
    barcode_capture_reset();

    // ---- Drift PID init ----
    PID drift; pid_init(&drift, DRIFT_KP, DRIFT_KI, DRIFT_KD, -DRIFT_CORR_MAX, +DRIFT_CORR_MAX);
    uint32_t left_total_prev = encoder_get_count(ENC_LEFT);
    uint32_t right_total_prev = encoder_get_count(ENC_RIGHT);

    while (true) {
        // --- periodic sampling ---
        if (absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {
            // Analog IR via line.c
            int ar = irsens_read_analog();
            uint16_t ir_raw = (ar < 0) ? 0 : (uint16_t)ar;

            // Schmitt edges (for barcode) from analog
            int new_state = analog_state;
            if (analog_state == 0 && ir_raw >= thr_high) new_state = 1;
            else if (analog_state == 1 && ir_raw <= thr_low) new_state = 0;
            if (new_state != analog_state) {
                uint64_t now_us = time_us_64();
                barcode_on_edge(analog_state, new_state, now_us);
                analog_state = new_state; d0_prev = new_state;
            } else { d0_age_ms += SAMPLE_IR_MS; }

            // D0 edges (hardware comparator)
            int d0_now = gpio_get(IR_D0_GPIO); uint64_t now_us = time_us_64();
            if (d0_now != d0_prev) { barcode_on_edge(d0_prev, d0_now, now_us); d0_prev = d0_now; }

            // End-of-pass finalize barcode
            if (capturing) {
                if (((time_us_64() - pass_last_edge_us) / 1000u) > BARCODE_IDLE_TIMEOUT_MS) {
                    char bits[MAX_BARS + 1] = {0};
                    int nbits = classify_widths_to_bits(black_width_us, black_count, bits, sizeof(bits));
                    if (nbits > 0) {
                        const char *which = match_pattern(bits);
                        printf("[BAR] count=%d bits=%s -> %s\n", nbits, bits, which);
                    } else {
                        printf("[BAR] no bits captured\n");
                    }
                    barcode_capture_reset();
                }
            }

            // ---------- Line Follow (edge-follow using IR1 analog) ----------
    #if LF_ENABLE
        // ---- Read dedicated line-follow sensor D0 (GP13) with short hold/debounce ----
        int lf_d0_now = gpio_get(IR2_D0_GPIO);   // usually 1=BLACK, 0=WHITE
        if (lf_prev_d0 < 0) { lf_prev_d0 = lf_d0_now; lf_state_hold_ms = 0; }
        else if (lf_d0_now != lf_prev_d0) {
            lf_state_hold_ms += SAMPLE_IR_MS;
            if (lf_state_hold_ms >= LF_D0_MIN_HOLD_MS) { lf_prev_d0 = lf_d0_now; }
            lf_d0_now = lf_prev_d0; // ignore flicker until hold time reached
        } else { lf_state_hold_ms = 0; }

        // Base error from digital sensor (decoupled from barcode)
    // --- Base error from digital sensor (decoupled from barcode)
    float err_raw = lf_d0_now ? +1.0f : -1.0f;     //  +1 when over black, -1 on white
#if LF_INVERT_STEER
    err_raw = -err_raw;
#endif

    // (Optional) analog micro-assist — gentle, won’t flip sign by itself
    if (CURVE_ASSIST_ALPHA > 0.0f) {
        float aerr = ((float)ir_raw - (float)threshold) / CURVE_ANALOG_EPS; // ~[-1..+1]
        if (aerr >  1.0f) aerr =  1.0f;
        if (aerr < -1.0f) aerr = -1.0f;
        err_raw = (1.0f - CURVE_ASSIST_ALPHA) * err_raw + CURVE_ASSIST_ALPHA * aerr;
    }

    // ====== Flip-guard hysteresis on steer SIGN ======
    int8_t want_sign = (err_raw >= 0.f) ? +1 : -1;
    if (lf_cmd_sign == 0) {
        lf_cmd_sign = want_sign;           // first time: lock immediately
        lf_flip_ms  = 0;
    } else if (want_sign != lf_cmd_sign) {
        lf_flip_ms += SAMPLE_IR_MS;        // new sign is being requested
        if (lf_flip_ms >= LF_FLIP_CONFIRM_MS) {
            lf_cmd_sign = want_sign;       // accept sign change after confirmation
            lf_flip_ms  = 0;
        }
    } else {
        lf_flip_ms = 0;                    // same sign: reset timer
    }

    // Magnitude follows err_raw, sign follows guarded lf_cmd_sign
    float mag = fabsf(err_raw);            // 0..1
    float err = (float)lf_cmd_sign * mag;  // final guarded error

    // PD on guarded error
    static float prev_err_f = 0.0f;
    float derr = err - prev_err_f; prev_err_f = err;

    float u = LF_KP * err + LF_KD * derr;           // ~[-1..+1]
    if (u > 1.0f) u = 1.0f; if (u < -1.0f) u = -1.0f;

    // Smooth & compute wheel targets (keep your existing smoothing/slew)
    u_filt = 0.75f * u_filt + 0.25f * u;

    float base = LF_BASE_SPEED_PCT;
    float turn_gain = LF_TURN_PCT;
    if (fabsf(u_filt) > 0.6f) { base -= 4.0f; turn_gain *= 1.10f; } // small auto-trim on sharp turns
    float turn = turn_gain * u_filt;

    // Optional: **mute drift-PID when turning hard** so it can’t fight the steer
    float drift_trim_pct = 0.0f;
#if DRIFT_PID_ENABLE
    {
        uint32_t l_now = encoder_get_count(ENC_LEFT);
        uint32_t r_now = encoder_get_count(ENC_RIGHT);
        int32_t diff_counts = (int32_t)r_now - (int32_t)l_now;
        drift_trim_pct = pid_update(&drift, 0.0f, (float)diff_counts);
        // fade out drift correction as steer grows
        float fade = 1.0f - fminf(1.0f, fabsf(u_filt));   // 1 at straight, →0 at hard turn
        drift_trim_pct *= fade;
    }
#endif

    float target_left  = base - turn + drift_trim_pct;
    float target_right = base + turn - drift_trim_pct;

    // torque floor + your existing slew limiter + motor_set_speed(...)
    if (target_left  < LF_MIN_WHEEL_PCT) target_left  = LF_MIN_WHEEL_PCT;
    if (target_right < LF_MIN_WHEEL_PCT) target_right = LF_MIN_WHEEL_PCT;

    float max_step = SLEW_PCT_PER_TICK;
    if (target_left  > left_cmd_pct  + max_step) target_left  = left_cmd_pct  + max_step;
    if (target_left  < left_cmd_pct  - max_step) target_left  = left_cmd_pct  - max_step;
    if (target_right > right_cmd_pct + max_step) target_right = right_cmd_pct + max_step;
    if (target_right < right_cmd_pct - max_step) target_right = right_cmd_pct - max_step;

    left_cmd_pct  = target_left;
    right_cmd_pct = target_right;

    motor_set_speed(left_cmd_pct/100.0f, right_cmd_pct/100.0f);

    #else
        motor_set_speed(0.35f, 0.35f);
    #endif



            next_sample = delayed_by_ms(next_sample, SAMPLE_IR_MS);
        }

        // --- telemetry print ---
        if (absolute_time_diff_us(get_absolute_time(), next_print) <= 0) {
            // close current D0 segment into duty window
            uint64_t now_us = time_us_64();
            uint32_t seg = (uint32_t)(now_us - d0_last_edge_us);
            if (d0_prev == 1) acc_black_us += seg; else if (d0_prev == 0) acc_white_us += seg;
            d0_last_edge_us = now_us;

            float black_ms = 0.f, white_ms = 0.f;
            float duty_black = d0_black_duty(&black_ms, &white_ms);

            // one more analog sample for display
            int ar = irsens_read_analog(); uint16_t ir_raw = (ar < 0) ? 0 : (uint16_t)ar;
            const char *state = (ir_raw >= threshold) ? "BLACK" : "WHITE";
            int d0_last = gpio_get(IR_D0_GPIO);

            // encoders
            uint32_t lticks = encoder_get_count(ENC_LEFT);
            uint32_t rticks = encoder_get_count(ENC_RIGHT);
            uint32_t lper   = encoder_get_period_us(ENC_LEFT);
            uint32_t rper   = encoder_get_period_us(ENC_RIGHT);
            float lrps = lper ? (1e6f / (float)lper) / (float)SLOTS_PER_REV : 0.f;
            float rrps = rper ? (1e6f / (float)rper) / (float)SLOTS_PER_REV : 0.f;

            printf("raw=%u thr=%u state=%s d0=%d "
                   "black_ms=%.1f white_ms=%.1f duty=%.1f%% age_ms=%u "
                   "L_ticks=%lu R_ticks=%lu L_rpm=%.2f R_rpm=%.2f "
                   "bars=%d\n",
                   ir_raw, threshold, state, d0_last,
                   black_ms, white_ms, duty_black, d0_age_ms,
                   (unsigned long)lticks, (unsigned long)rticks,
                   lrps * 60.f, rrps * 60.f,
                   black_count);

            d0_duty_reset();
            next_print = delayed_by_ms(next_print, PRINT_MS);
        }

        tight_loop_contents();
    }
}
