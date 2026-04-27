#ifndef INTEGRATED_LCD_DRIVER_H
#define INTEGRATED_LCD_DRIVER_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

HAL_StatusTypeDef lcd_driver_init(I2C_HandleTypeDef *hi2c,
                                  GPIO_TypeDef *reset_port,
                                  uint16_t reset_pin);
HAL_StatusTypeDef lcd_driver_write_lines(const char *line0, const char *line1);
bool lcd_driver_is_ready(void);

#endif /* INTEGRATED_LCD_DRIVER_H */
