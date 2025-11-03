#include <stdio.h>
#include "pico/stdlib.h"
#include "encoder.h"

#ifndef ENCODER_GPIO
#define ENCODER_GPIO 2
#endif

#ifndef SLOTS_PER_REV
#define SLOTS_PER_REV 20
#endif

int main() {
    stdio_init_all();
    sleep_ms(1500);

    encoder_init(ENCODER_GPIO);

    uint32_t last_count = 0;
    const uint32_t interval_ms = 200;

    while (true) {
        sleep_ms(interval_ms);

        uint32_t count = encoder_get_count();
        uint32_t delta = count - last_count;
        last_count = count;

        // Speed estimate A: from last measured period
        uint32_t period_us = encoder_get_period_us();
        float rps_period = 0.0f;
        if (period_us > 0) {
            float edges_per_sec = 1e6f / (float)period_us; // pulses/s
            rps_period = edges_per_sec / (float)SLOTS_PER_REV;
        }

        // Speed estimate B: from delta over window
        float rps_delta = ((float)delta / (float)SLOTS_PER_REV) / (interval_ms / 1000.0f);

        printf("ticks=%lu period_us=%lu rps(period)=%.3f rps(delta)=%.3f rpm=%.1f\n",
               (unsigned long)count,
               (unsigned long)period_us,
               rps_period,
               rps_delta,
               rps_period * 60.0f);
    }
}
