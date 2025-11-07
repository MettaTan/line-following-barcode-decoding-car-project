#pragma once
#include "pico/stdlib.h"

// Identify which wheel
typedef enum { ENC_LEFT = 0, ENC_RIGHT = 1 } enc_id_t;

#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV 20u
#endif

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


/** Wheel encoder data struct */
typedef struct {
    float left_speed;
    float right_speed;
    float left_distance;
    float right_distance;
    float avg_speed;
    float avg_distance;
} EncoderData;

EncoderData encoder_get_data(float wheel_diameter_mm);
/** Rotational speed in revolutions per second */
float encoder_get_speed_rps(enc_id_t id);

/** Linear speed in millimeters per second (requires wheel diameter) */
float encoder_get_speed_mm_s(enc_id_t id, float wheel_diameter_mm);

/** Total distance travelled by a single wheel in millimeters */
float encoder_get_distance_mm(enc_id_t id, float wheel_diameter_mm);

/** Average distance travelled by both wheels in millimeters */
float encoder_get_average_distance_mm(float wheel_diameter_mm);
