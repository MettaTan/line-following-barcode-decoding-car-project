#include "line.h"
#include "hardware/adc.h"

// ADC pin for IR sensor
static uint8_t adc_input = 0;

// Analog IR sensor initialization
void irsens_init(uint adc_gpio) {
    adc_init();
    adc_gpio_init(adc_gpio);

    // Conversion of GPIO to ADC
    if (adc_gpio == 26) {
        adc_input = 0;
    } else if (adc_gpio == 27) {
        adc_input = 1;
    } else if (adc_gpio == 28) {
        adc_input = 2;
    } else {
        adc_input = 0xFF;  // Invalid GPIO for ADC
    }
}

int irsens_read_analog(void) {
    if (adc_input == 0xFF) {
        return LINE_EINVAL;  // Invalid ADC pin
    }

    adc_select_input(adc_input);
    return adc_read();  // Range 0 - 4095
}