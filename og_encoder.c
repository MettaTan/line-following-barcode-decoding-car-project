#include "encoder.h"
#include "hardware/gpio.h"

static uint     enc_gpio[2]               = {0xFFFFFFFFu, 0xFFFFFFFFu};
static volatile uint32_t enc_count[2]     = {0, 0};
static volatile uint32_t enc_period_us[2] = {0, 0};
static volatile uint64_t enc_last_us[2]   = {0, 0};

static const uint32_t DEBOUNCE_US = 300; // ignore edges within this gap

static inline int id_from_gpio(uint gpio) {
    if (gpio == enc_gpio[ENC_LEFT])  return ENC_LEFT;
    if (gpio == enc_gpio[ENC_RIGHT]) return ENC_RIGHT;
    return -1;
}

static void encoder_irq(uint gpio, uint32_t events) {
    int id = id_from_gpio(gpio);
    if (id < 0) return;

    uint64_t now = time_us_64();
    uint64_t dt  = now - enc_last_us[id];
    if (dt >= DEBOUNCE_US) {
        enc_count[id] += 1;
        enc_period_us[id] = (uint32_t)dt;
        enc_last_us[id] = now;
    }
}

static void one_init(uint gpio, enc_id_t id) {
    enc_gpio[id] = gpio;
    gpio_init(gpio);
    gpio_set_dir(gpio, false);     // input
    gpio_pull_up(gpio);            // most modules are open-collector/phototransistor
    gpio_set_irq_enabled_with_callback(gpio, GPIO_IRQ_EDGE_RISE, true, &encoder_irq);
    enc_last_us[id] = time_us_64();
}

void encoder_init_pair(uint left_gpio, uint right_gpio) {
    // First call sets the shared callback; the 2nd attaches to same handler.
    if (left_gpio  != 0xFFFFFFFFu) one_init(left_gpio,  ENC_LEFT);
    if (right_gpio != 0xFFFFFFFFu) one_init(right_gpio, ENC_RIGHT);
}

uint32_t encoder_get_count(enc_id_t id)     { return enc_count[id]; }
uint32_t encoder_get_period_us(enc_id_t id) { return enc_period_us[id]; }
void     encoder_reset(enc_id_t id)         { enc_count[id] = 0; enc_period_us[id] = 0; }
