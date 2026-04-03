#ifndef DOWNLINK_INPUT_SOURCE_SELECTOR_H
#define DOWNLINK_INPUT_SOURCE_SELECTOR_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

typedef enum
{
    DOWNLINK_INPUT_SOURCE_USB = 0,
    DOWNLINK_INPUT_SOURCE_XBEE,
} DownlinkInputSource;

void input_source_selector_init(void);
void input_source_selector_poll(void);

DownlinkInputSource downlink_input_source_get_current(void);
const char *downlink_input_source_get_current_name(void);
UART_HandleTypeDef *downlink_input_source_get_active_uart(void);
bool downlink_input_source_is_selected_uart(const UART_HandleTypeDef *huart);

#endif /* DOWNLINK_INPUT_SOURCE_SELECTOR_H */
