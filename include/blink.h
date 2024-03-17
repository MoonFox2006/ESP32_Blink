#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/gpio.h>

#define BLINK_TIME  25 // 25 ms.

typedef enum { BLINK_UNDEFINED, BLINK_OFF, BLINK_ON, BLINK_TOGGLE, BLINK_4HZ, BLINK_2HZ, BLINK_1HZ, BLINK_FADEIN, BLINK_FADEOUT, BLINK_BREATH } blink_t;

esp_err_t blink_init();
esp_err_t blink_add(gpio_num_t pin, bool level, uint8_t *index);
esp_err_t blink_update(uint8_t index, blink_t mode, uint16_t bright);

#ifdef __cplusplus
}
#endif
