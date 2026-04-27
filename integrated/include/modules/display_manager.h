#ifndef INTEGRATED_DISPLAY_MANAGER_H
#define INTEGRATED_DISPLAY_MANAGER_H

#include "stm32f4xx_hal.h"

void display_manager_init(I2C_HandleTypeDef *hi2c);
void display_manager_wait_startup_done(void);
void display_manager_poll(void);
void display_manager_show_error(void);

#endif /* INTEGRATED_DISPLAY_MANAGER_H */
