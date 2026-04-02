#ifndef UPLINK_STATUS_LEDS_H
#define UPLINK_STATUS_LEDS_H

#include "stm32f4xx_hal.h"

void status_leds_init(void);
void status_leds_poll(void);
void status_leds_on_rx_activity(void);
void status_leds_on_tim_period_elapsed(TIM_HandleTypeDef *htim);

#endif /* UPLINK_STATUS_LEDS_H */
