#ifndef LINE_H
#define LINE_H

#include <pico/stdlib.h>

// Return codes
#define LINE_OK 0
#define LINE_EINVAL -1

// Functions
void irsens_init(uint adc_gpio);

// Reading of analaog value from the IR sensor
int irsens_read_analog(void);

#endif