#pragma once
#include "pico/stdlib.h"

// Identify which wheel
typedef enum { ENC_LEFT = 0, ENC_RIGHT = 1 } enc_id_t;

/**
 * Initialize two single-channel encoders (one pin per wheel).
 * Pass the GPIO for the left and right encoder OUT signals.
 * If you only have one encoder, pass 0xFFFFFFFF for the unused side.
 */
void encoder_init_pair(uint left_gpio, uint right_gpio);

// Backwards-compat wrapper (kept for older code paths)
static inline void encoder_init(uint gpio) { encoder_init_pair(gpio, 0xFFFFFFFFu); }

/** Monotonic tick count (edges) seen on that wheel */
uint32_t encoder_get_count(enc_id_t id);

/** Microseconds between the last two edges on that wheel (0 if unknown yet) */
uint32_t encoder_get_period_us(enc_id_t id);

/** Clear counters/periods for that wheel */
void encoder_reset(enc_id_t id);
