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
#include "mqtt_client.h"
#include "pico/cyw43_arch.h"     
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "lwip/ip_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"

// -------------------- Pin map (MUST MATCH WIRING) --------------------
// Line-follow analog sensor (IR1) stays on ADC0
#define LF_ADC_GPIO       26      // A0 -> GP26/ADC0  (used for LF + telemetry)

// Barcode sensor moved to the other unit's D0 (digital pulses only)
#define BAR_D0_GPIO       13      // D0 -> GP13 (barcode edges)

// (Kept for compatibility if other modules include these, but not used for barcode now)
#define IR_D0_GPIO        27      // old D0; not used for barcode anymore

#define ENC_LEFT_GPIO      2      // Left wheel encoder OUT -> GP2
#define ENC_RIGHT_GPIO     4      // Right wheel encoder OUT -> GP4

// Keep your motor pins / wiring (used by motor.c)
#define M1A                8
#define M1B                9
#define M2A               10
#define M2B               11

// -------------------- Timings / params --------------------
#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV      20
#endif

#define SAMPLE_IR_MS        2      // fast loop helps both barcode and LF
#define PRINT_MS          250
#define CALIB_MS          800

// --------- DRIVE/LF toggles ---------
#define LF_ENABLE           1
#define LF_BASE_SPEED_PCT  36.0f   // was 32
#define LF_TURN_PCT        34.0f   // was 28
#define LF_KP              0.0052f // was 0.0035f (more authority per ADC count)
#define LF_KD              0.0000f // kill derivative for now

// --- Anti-jitter knobs for LF ---
#define LF_ERR_DEADBAND     15      // was 25 (more sensitivity near edge)
#define IR_EMA_ALPHA        0.25f   // was 0.18 (snappier)
#define LF_U_SMOOTH_ALPHA   0.12f   // was 0.18 (less lag)
#define SLEW_PCT_PER_TICK    6.0f   // was 3.0 (can actually turn)
#define MIN_WHEEL_PCT       18.0f

// -------- Line-follow recovery (missing in your file) --------
#ifndef LF_LOST_MS
#define LF_LOST_MS        450     // ms on WHITE before we attempt recovery
#endif
#ifndef LF_RECOVER_PCT
#define LF_RECOVER_PCT    22.0f   // motor % while spinning to re-acquire line
#endif

// --- WHITE-reverse assist ---
#define WHITE_REV_ENABLE         0   // <-- disable first to verify LF recovers on curves
// If you want it later: set to 1 and use:
#define WHITE_REV_MIN_MS       120
#define WHITE_REV_MAX_MS       280
#define WHITE_REV_PCT           18.0f
#define WHITE_REV_TURN_PCT      26.0f
#define WHITE_REV_SLEW           6.0f


// -------------------- Barcode capture --------------------
#define BARCODE_IDLE_TIMEOUT_MS   1500
#define MAX_BARS                  64
#define MIN_SEPARATION_RATIO      1.35f

// Editable reference patterns (your sheets)
#define A_PATTERN "001101000100110"
#define Z_PATTERN "001100110000110"

// -------------------- D0 duty/width trackers (barcode) --------------------
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

static void barcode_capture_reset(void) {
    black_count = 0; capturing = false;
    pass_last_edge_us = time_us_64();
    black_start_us = 0;
}
static void barcode_on_edge(int prev_level, int now_level, uint64_t now_us) {
    // duration of the level that just ended
    uint32_t dt = (uint32_t)(now_us - d0_last_edge_us);

    // accumulate duty
    if (prev_level == 1) acc_black_us += dt;
    else if (prev_level == 0) acc_white_us += dt;

    // publish the edge event to MQTT (level that ended + its duration)
    mqtt_publish_barcode_edge(prev_level, dt);

    d0_last_edge_us = now_us;
    d0_age_ms = 0;

    // Start capture on first ever edge
    if (!capturing) {
        capturing = true;
        black_count = 0;
        black_start_us = 0;
    }

    // Record bars
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

// -------------------- LF smoothing state --------------------
static float ir_raw_ema   = 0.0f;
static float u_filt       = 0.0f;
static float left_cmd_pct = 0.0f, right_cmd_pct = 0.0f;

// -------------------- IR thresholding (for LF telemetry only) --------------------
static int      analog_state    = 0;   // not used for barcode anymore
static uint16_t thr_low         = 0, thr_high = 0;

// -------------------- Drift PID (encoders) --------------------
typedef struct { float kp, ki, kd, integ, prev, out_min, out_max; } PID;
static inline void pid_init(PID* p, float kp, float ki, float kd, float mn, float mx){ p->kp=kp;p->ki=ki;p->kd=kd;p->integ=0;p->prev=0;p->out_min=mn;p->out_max=mx; }
static inline float pid_update(PID* p, float setpoint, float current){
    float e = setpoint - current; p->integ += e; float d = e - p->prev; p->prev = e;
    float u = p->kp*e + p->ki*p->integ + p->kd*d;
    if (u > p->out_max) u = p->out_max; else if (u < p->out_min) u = p->out_min;
    return u;
}

// ============================================================
// Line Event Publisher (on-track / off-track)
// Publishes only the line event, no raw data
// ============================================================
void mqtt_publish_line_event(bool on_track) {
    if (!mqtt_is_connected()) return;

    const char *event = on_track ? "ON_TRACK" : "OFF_TRACK";

    char payload[64];
    snprintf(payload, sizeof(payload),
             "{"
               "\"line_event\": \"%s\""
             "}",
             event);

    mqtt_publish_message("telemetry/line", payload);
}


int main(void) {
    stdio_init_all();
        if (cyw43_arch_init()) {
        printf("❌ CYW43 init failed.\n");
        return -1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("❌ Wi-Fi connect failed.\n");
        return -1;
    }
    printf("✅ Wi-Fi connected.\n");

    printf("Waiting for IP...\n");
    for (int i = 0; i < 20; i++) {
        if (netif_default && netif_is_up(netif_default)) {
            const ip4_addr_t *ip = netif_ip4_addr(netif_default);
            printf("🌐 Pico IP address: %s\n", ip4addr_ntoa(ip));
            break;
        }
    }

    printf("Starting MQTT...\n");
    mqtt_client_start("172.20.10.13", 1883);

    // Wait for MQTT connection
    for (int i = 0; i < 10; i++) {
        // pico_cyw43_arch_lwip_threadsafe_background();
        mqtt_loop_poll();          // actively drives the MQTT handshake
        if (mqtt_is_connected()) {
            printf("✅ MQTT connected successfully before main loop.\n");
            break;
        }
    sleep_ms(100);
}
    sleep_ms(1200);

    printf("\n=== LF on GP26(ADC0) + BARCODE on GP13(D0) — with WHITE-REVERSE assist ===\n");

    // ---- Line-follow analog (ADC0) ----
    irsens_init(LF_ADC_GPIO);

    // ---- Barcode D0 (new pin GP13) ----
    gpio_init(BAR_D0_GPIO); gpio_set_dir(BAR_D0_GPIO, false); gpio_pull_down(BAR_D0_GPIO);

    // ---- Encoders ----
    encoder_init_pair(ENC_LEFT_GPIO, ENC_RIGHT_GPIO);

    // ---- Motors ----
    motor_init(M1A, M1B, M2A, M2B);
    motor_stop();

    // ---- IMU (preserve structure) ----
    imu_init();

    // ---- Auto-calibrate IR threshold from analog A0 (for LF only) ----
    uint32_t t0 = to_ms_since_boot(get_absolute_time());
    uint16_t minv = 0xFFFF, maxv = 0;
    while (to_ms_since_boot(get_absolute_time()) - t0 < CALIB_MS) {
        int ar = irsens_read_analog(); uint16_t v = (ar < 0) ? 0 : (uint16_t)ar;
        if (v < minv) minv = v; if (v > maxv) maxv = v; sleep_ms(5);
    }
    uint16_t span = (maxv > minv) ? (maxv - minv) : 0;
    uint16_t threshold = (span >= 60) ? (uint16_t)(minv + span / 2) : 600;
    printf("[CAL] min=%u max=%u span=%u thr=%u\n", minv, maxv, span, threshold);

    // Schmitt band ~10% of threshold, min 60 counts (telemetry only)
    uint16_t band = (uint16_t)(threshold / 10); if (band < 60) band = 60;
    thr_low  = (threshold > band) ? (threshold - band) : (threshold / 2);
    thr_high = threshold + band;

    // Seed for EMA
    int ar0 = irsens_read_analog(); analog_state = (ar0 >= threshold) ? 1 : 0;
    ir_raw_ema = (float)((ar0 < 0) ? 0 : ar0);

    // ---- Loop timers ----
    absolute_time_t next_sample = make_timeout_time_ms(SAMPLE_IR_MS);
    absolute_time_t next_print  = make_timeout_time_ms(PRINT_MS);

    // ---- Init barcode duty/edges on new D0 ----
    d0_prev = gpio_get(BAR_D0_GPIO);
    d0_last_edge_us = time_us_64();
    d0_duty_reset();
    barcode_capture_reset();

    // ---- Drift PID init ----
    PID drift; pid_init(&drift, 0.003f, 0.00001f, 0.0005f, -20.0f, +20.0f);
    (void)drift; // silence unused if DRIFT disabled by macro

    while (true) {
        // --- periodic sampling ---
        if (absolute_time_diff_us(get_absolute_time(), next_sample) <= 0) {
            // LINE-FOLLOW analog read (ADC0 GP26)
            int ar = irsens_read_analog();
            uint16_t ir_raw = (ar < 0) ? 0 : (uint16_t)ar;

            // ----------- BARCODE: D0-only edges on BAR_D0_GPIO -----------
            {
                int d0_now = gpio_get(BAR_D0_GPIO);
                uint64_t now_us = time_us_64();
                if (d0_now != d0_prev) {               // edge detected
                    barcode_on_edge(d0_prev, d0_now, now_us);
                    uint32_t dt = (uint32_t)(now_us - d0_last_edge_us); // d0_last_edge_us was updated inside, so capture before call if needed
                    // Simpler: recalc using your local time vars:
                    d0_prev = d0_now;
                } else {
                    // age the current segment for duty readout
                    d0_age_ms += SAMPLE_IR_MS;
                }
            }

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

                    // After you build `bits` and before you reset:
                    const char *which = match_pattern(bits);     // strict match (A/Z/UNKNOWN)

                    // Soft guess: Hamming distance to A/Z; report side + confidence
                    int hA = 0, hZ = 0;
                    int na = strlen(A_PATTERN), nz = strlen(Z_PATTERN);
                    int n  = strlen(bits);

                    // Compare against windows if counts differ (take best aligned slice)
                    int bestA = 99, bestZ = 99;
                    for (int off = 0; off + na <= n; ++off) {
                        int d = 0; for (int i=0;i<na;i++) d += (bits[off+i] != A_PATTERN[i]);
                        if (d < bestA) bestA = d;
                    }
                    for (int off = 0; off + nz <= n; ++off) {
                        int d = 0; for (int i=0;i<nz;i++) d += (bits[off+i] != Z_PATTERN[i]);
                        if (d < bestZ) bestZ = d;
                    }
                    // Confidence = how much better the best match is vs the other (0..1)
                    float conf = 0.0f;
                    const char *g = "UNKNOWN";
                    if (bestA < 99 || bestZ < 99) {
                        int maxLen = (bestA < 99) ? na : nz;
                        int worse  = (bestA <= bestZ) ? bestZ : bestA;
                        int better = (bestA <= bestZ) ? bestA : bestZ;
                        float span = (float)maxLen;           // normalize by pattern length
                        float sep  = (float)(worse - better); // how much better
                        conf = (sep <= 0) ? 0.0f : fminf(1.0f, sep / span);
                        g = (better == bestA) ? "RIGHT" : "LEFT";   // A=RIGHT, Z=LEFT
                    }
                    if (strcmp(which,"A (RIGHT)")==0) { g="RIGHT"; conf=1.0f; }
                    else if (strcmp(which,"Z (LEFT)")==0) { g="LEFT"; conf=1.0f; }

                    // Print as before
                    printf("[BAR] count=%d bits=%s -> %s (guess=%s conf=%.2f)\n",
                        nbits, bits, which, g, conf);

                    // Publish snapshot to MQTT regardless of strict match
                    mqtt_publish_barcode_snapshot(bits, nbits, g, conf);

                    barcode_capture_reset();
                }
            }

            // ------------------- Line Follow (analog) -------------------
#if LF_ENABLE
            // Smooth analog
            ir_raw_ema = (1.0f - IR_EMA_ALPHA) * ir_raw_ema + IR_EMA_ALPHA * (float)ir_raw;

            // Error relative to threshold
            int32_t err = (int32_t)ir_raw_ema - (int32_t)threshold;

            // deadband around edge
            if (err > -LF_ERR_DEADBAND && err < LF_ERR_DEADBAND) err = 0;

            // PD
            static int32_t prev_err_pd = 0;
            int32_t derr = err - prev_err_pd; 
            prev_err_pd = err;

            float u = LF_KP * (float)err;
            if (u >  1.0f) u =  1.0f;
            if (u < -1.0f) u = -1.0f;

            // smooth steering
           u_filt = (1.0f - LF_U_SMOOTH_ALPHA) * u_filt + LF_U_SMOOTH_ALPHA * u;

            // --- track WHITE streak time ---
            static uint32_t lf_white_ms = 0;
            if (ir_raw < threshold) lf_white_ms += SAMPLE_IR_MS; else lf_white_ms = 0;

            // Optional: encoder drift trim (fade when turning)
            float drift_trim_pct = 0.0f;
            #if DRIFT_PID_ENABLE
            {
                uint32_t l_now = encoder_get_count(ENC_LEFT);
                uint32_t r_now = encoder_get_count(ENC_RIGHT);
                int32_t diff_counts = (int32_t)r_now - (int32_t)l_now;
                drift_trim_pct = pid_update(&drift, 0.0f, (float)diff_counts);
                float fade = 1.0f - fminf(1.0f, fabsf(u_filt)); // 1 straight -> 0 hard turn
                drift_trim_pct *= fade;
            }
            #endif

            // --------- WHITE-REVERSE assist ---------
#if WHITE_REV_ENABLE
            bool do_white_rev = (lf_white_ms >= WHITE_REV_MIN_MS && lf_white_ms <= WHITE_REV_MAX_MS);
#else
            bool do_white_rev = false;
#endif

            if (do_white_rev) {
                // Reverse while turning back toward the line.
                // In reverse, invert steering mapping so it arcs inward.
                float turn = WHITE_REV_TURN_PCT * u_filt;
                float target_left  = -WHITE_REV_PCT - turn + drift_trim_pct; // note minus base
                float target_right = -WHITE_REV_PCT + turn - drift_trim_pct;

                // floors are irrelevant in reverse; still slew-limit to avoid jerks
                float max_step = WHITE_REV_SLEW;
                if (target_left  > left_cmd_pct  + max_step) target_left  = left_cmd_pct  + max_step;
                if (target_left  < left_cmd_pct  - max_step) target_left  = left_cmd_pct  - max_step;
                if (target_right > right_cmd_pct + max_step) target_right = right_cmd_pct + max_step;
                if (target_right < right_cmd_pct - max_step) target_right = right_cmd_pct - max_step;

                left_cmd_pct  = target_left;
                right_cmd_pct = target_right;

                motor_set_speed(left_cmd_pct/100.0f, right_cmd_pct/100.0f);
            }
            else if (lf_white_ms > LF_LOST_MS) {
                // long WHITE -> recovery spin
                motor_set_speed(LF_RECOVER_PCT/100.0f, -LF_RECOVER_PCT/100.0f);
            } else {
                // normal forward LF
                float base = LF_BASE_SPEED_PCT;
                float turn = LF_TURN_PCT * u_filt;

                float target_left  = base - turn + drift_trim_pct;
                float target_right = base + turn - drift_trim_pct;

                // keep wheels turning
                if (target_left  < MIN_WHEEL_PCT) target_left  = MIN_WHEEL_PCT;
                if (target_right < MIN_WHEEL_PCT) target_right = MIN_WHEEL_PCT;

                // slew limiter (faster now)
                float max_step = SLEW_PCT_PER_TICK;
                if (target_left  > left_cmd_pct  + max_step) target_left  = left_cmd_pct  + max_step;
                if (target_left  < left_cmd_pct  - max_step) target_left  = left_cmd_pct  - max_step;
                if (target_right > right_cmd_pct + max_step) target_right = right_cmd_pct + max_step;
                if (target_right < right_cmd_pct - max_step) target_right = right_cmd_pct - max_step;

                left_cmd_pct  = target_left;
                right_cmd_pct = target_right;

                motor_set_speed(left_cmd_pct/100.0f, right_cmd_pct/100.0f);
            }
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

            // analog sample for display
            int ar = irsens_read_analog(); uint16_t ir_raw = (ar < 0) ? 0 : (uint16_t)ar;
            const char *state = (ir_raw >= threshold) ? "BLACK" : "WHITE";
            int d0_last = gpio_get(BAR_D0_GPIO); // show new barcode D0

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

            bool on_track = (ir_raw >= threshold);
            mqtt_publish_line_event(on_track);
            mqtt_publish_barcode_duty(black_ms, white_ms, duty_black);


            // --- MQTT telemetry ---
            mqtt_publish_imu();          // publishes raw + filtered IMU
            mqtt_publish_speed();        // publishes avg wheel speed
            mqtt_publish_distance();     // publishes total distance

            d0_duty_reset();
            next_print = delayed_by_ms(next_print, PRINT_MS);
        }

        tight_loop_contents();
    }
}
