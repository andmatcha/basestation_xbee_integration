#ifndef DOWNLINK_STATUS_LEDS_H
#define DOWNLINK_STATUS_LEDS_H

#include "stm32f4xx_hal.h"

void status_leds_init(void);
void status_leds_poll(void);
void status_leds_on_rx_activity(void);
void status_leds_on_tim_pwm_pulse_finished(TIM_HandleTypeDef *htim);

#endif /* DOWNLINK_STATUS_LEDS_H */
