#ifndef DOWNLINK_RGB_LED_DRIVER_H
#define DOWNLINK_RGB_LED_DRIVER_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_led_color_t;

void rgb_led_driver_init(void);
void rgb_led_driver_poll(void);
void rgb_led_driver_set_colors(rgb_led_color_t mode_led,
                               rgb_led_color_t state_led);
void rgb_led_driver_on_tim_period_elapsed(TIM_HandleTypeDef *htim);

#endif /* DOWNLINK_RGB_LED_DRIVER_H */
