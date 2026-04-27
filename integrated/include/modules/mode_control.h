#ifndef INTEGRATED_MODE_CONTROL_H
#define INTEGRATED_MODE_CONTROL_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MODULE_MODE_ARM = 0,
    MODULE_MODE_SCIENCE,
} module_mode_t;

typedef enum
{
    XBEE_MODE_ONBOARD = 0,
    XBEE_MODE_EXTERNAL,
} xbee_mode_t;

void mode_control_init(void);
void mode_control_poll(void);

module_mode_t mode_control_get_module_mode(void);
xbee_mode_t mode_control_get_xbee_mode(void);
uint32_t mode_control_get_generation(void);

const char *mode_control_get_module_name(void);
const char *mode_control_get_xbee_name(void);
UART_HandleTypeDef *mode_control_get_active_xbee_uart(void);
bool mode_control_is_active_xbee_uart(const UART_HandleTypeDef *huart);

#endif /* INTEGRATED_MODE_CONTROL_H */
