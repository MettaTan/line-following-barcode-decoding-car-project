#include "motor.h"
#include <hardware/pwm.h>
#include "pico/stdlib.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define PWM_WRAP    1000u
#define PWM_DIVIDER 6.25f

static uint m1a, m1b;  // Motor A 
static uint m2a, m2b;  // Motor B 

// --- Helper to initialize a PWM pin ---
static void pwm_init_pin(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, PWM_DIVIDER);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(gpio, 0);
}

// --- Initialize motor pins (direction + PWM) ---
void motor_init(uint m1a_pin, uint m1b_pin, uint m2a_pin, uint m2b_pin) {
    m1a = m1a_pin;
    m1b = m1b_pin;
    m2a = m2a_pin;
    m2b = m2b_pin;


    // PWM pins
    pwm_init_pin(m1a);
    pwm_init_pin(m1b);
    pwm_init_pin(m2a);
    pwm_init_pin(m2b);

    motor_stop();
}


// Convert speed fraction [-1,1] to PWM duty [0, PWM_WRAP]
static inline uint16_t duty_from_float(float s) {
    if (s < 0.0f) s = -s;
    if (s > 1.0f) s = 1.0f;
    return (uint16_t)(s * PWM_WRAP);
}


// --- Set motor speed (0–100%) ---
void motor_set_speed(float left, float right) {
  uint16_t d1 = duty_from_float(left);
  uint16_t d2 = duty_from_float(right);


   if (left > 0) {         // Forward
        pwm_set_gpio_level(m1a, d1);
        pwm_set_gpio_level(m1b, 0);
    } else if (left < 0) {  // Reverse
        pwm_set_gpio_level(m1a, 0);
        pwm_set_gpio_level(m1b, d1);
    } else {                // Stop
        pwm_set_gpio_level(m1a, 0);
        pwm_set_gpio_level(m1b, 0);
    }

    if (right > 0) {        // Forward
        pwm_set_gpio_level(m2a, d2);
        pwm_set_gpio_level(m2b, 0);
    } else if (right < 0) { // Reverse
        pwm_set_gpio_level(m2a, 0);
        pwm_set_gpio_level(m2b, d2);
    } else {                // Stop
        pwm_set_gpio_level(m2a, 0);
        pwm_set_gpio_level(m2b, 0);
    }
}

// --- Stop both motors ---
void motor_stop(void) {
    pwm_set_gpio_level(m1a, 0);
    pwm_set_gpio_level(m1b, 0);
    pwm_set_gpio_level(m2a, 0);
    pwm_set_gpio_level(m2b, 0);
}

void move_robot(const char *action, uint32_t duration_ms, bool ramp, float speed) {
    float left = 0, right = 0;
    float normalized_speed = speed / 100.0f; // Convert 0-100 to 0.0-1.0

    if      (strcmp(action, "forward") == 0)  { left = +normalized_speed; right = +normalized_speed; }
    else if (strcmp(action, "backward") == 0) { left = -normalized_speed; right = -normalized_speed; }
    else if (strcmp(action, "left") == 0)     { left = +normalized_speed; right = -normalized_speed; }
    else if (strcmp(action, "right") == 0)    { left = -normalized_speed;  right = +normalized_speed; }
    else if (strcmp(action, "stop") == 0)     { left = 0;   right = 0; }

    const uint32_t step_ms = 100;
    uint32_t elapsed = 0;

    while (elapsed < duration_ms) {
        float phase = ramp ? ((float)elapsed / (float)duration_ms) : 1.0f;
        float scale = ramp ? (0.3f + 0.7f * phase) : 1.0f;

        motor_set_speed(left * scale, right * scale);
        sleep_ms(step_ms);
        elapsed += step_ms;

        printf("[MOVE] %s: L=%.1f R=%.1f\n", action, left * scale, right * scale);
    }

    motor_stop();
}

void move_robot_custom(uint32_t duration_ms, bool ramp, float left_speed, float right_speed) {
    // left_speed and right_speed: 0-100 range
    float left = left_speed / 100.0f;
    float right = right_speed / 100.0f;

    const uint32_t step_ms = 100;
    uint32_t elapsed = 0;

    while (elapsed < duration_ms) {
        float phase = ramp ? ((float)elapsed / (float)duration_ms) : 1.0f;
        float scale = ramp ? (0.3f + 0.7f * phase) : 1.0f;

        motor_set_speed(left * scale, right * scale);
        sleep_ms(step_ms);
        elapsed += step_ms;

        printf("[MOVE] Custom: L=%.1f R=%.1f\n", left * scale, right * scale);
    }

    motor_stop();
}