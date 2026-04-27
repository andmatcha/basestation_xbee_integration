#ifndef DOWNLINK_INPUT_SOURCE_SELECTOR_H
#define DOWNLINK_INPUT_SOURCE_SELECTOR_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

void input_source_selector_init(void);
void input_source_selector_poll(void);

const char *downlink_input_source_get_current_name(void);
UART_HandleTypeDef *downlink_input_source_get_active_uart(void);
bool downlink_input_source_is_selected_uart(const UART_HandleTypeDef *huart);
bool downlink_input_source_is_science_mode_enabled(void);

#endif /* DOWNLINK_INPUT_SOURCE_SELECTOR_H */
