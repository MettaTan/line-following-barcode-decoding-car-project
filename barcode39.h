#pragma once
#include <stdint.h>
#include <stdbool.h>

// Tuning knobs (safe defaults)
#ifndef BARCODE_IDLE_TIMEOUT_MS
#define BARCODE_IDLE_TIMEOUT_MS   2200   // end of pass if no edges for this long
#endif
#ifndef BARCODE_MAX_ELEMS
#define BARCODE_MAX_ELEMS         256    // bars + spaces collected in one pass
#endif
#ifndef BARCODE_MIN_RATIO_WIDE
#define BARCODE_MIN_RATIO_WIDE    1.6f   // wide is >= 1.6 × narrow (tolerant)
#endif
#ifndef BARCODE_MAX_RATIO_WIDE
#define BARCODE_MAX_RATIO_WIDE    3.6f   // wide is <= 3.6 × narrow
#endif

// Call on every edge from the BAR_D0 pin; level is 1=black(bar), 0=white(space)
void barcode39_on_edge(int level_now, uint64_t t_us);

// Call frequently; if the pass is complete (idle timeout), returns true and
// writes the decoded ASCII (without the start/stop) into out (NUL-terminated).
// If MOD43 is enabled in future, this will also validate it.
bool barcode39_try_finalize(uint64_t now_us, char *out, int out_sz, float *confidence);

// Duty/edge telemetry helpers (unchanged external behavior)
void barcode39_get_duty_ms(float *black_ms, float *white_ms, float *duty_pct, uint32_t *age_ms);
void barcode39_reset(void);

// Optional: expose capturing state to slow the car while reading
bool barcode39_is_capturing(void);
