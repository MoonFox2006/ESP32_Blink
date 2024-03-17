#include <Arduino.h>
#include "blink.h"

constexpr uint8_t LED_PIN = 15;
constexpr bool LED_LEVEL = HIGH;

static const char *MODES[] = {
  "!", "OFF", "ON", "TOGGLE", "4HZ", "2HZ", "1HZ", "FADEIN", "FADEOUT", "BREATH"
};

blink_t blink = BLINK_ON;

void setup() {
  Serial.begin(115200);

  ESP_ERROR_CHECK(blink_init());
  ESP_ERROR_CHECK(blink_add((gpio_num_t)LED_PIN, LED_LEVEL, NULL));
}

void loop() {
  Serial.printf("LED %s\n", MODES[blink]);
#if defined(CONFIG_IDF_TARGET_ESP32)
  ESP_ERROR_CHECK(blink_update(0, blink, 255));
#else
  ESP_ERROR_CHECK(blink_update(0, blink, 256));
#endif
  delay(5000);
  if (++*(uint8_t*)&blink > BLINK_BREATH)
    blink = BLINK_OFF;
}
