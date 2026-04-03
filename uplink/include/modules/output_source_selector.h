#ifndef UPLINK_OUTPUT_SOURCE_SELECTOR_H
#define UPLINK_OUTPUT_SOURCE_SELECTOR_H

#include "stm32f4xx_hal.h"

typedef enum
{
    UPLINK_OUTPUT_SOURCE_USB = 0,
    UPLINK_OUTPUT_SOURCE_XBEE,
} UplinkOutputSource;

void output_source_selector_init(void);
void output_source_selector_poll(void);

UplinkOutputSource uplink_output_source_get_current(void);
const char *uplink_output_source_get_current_name(void);
UART_HandleTypeDef *uplink_output_source_get_active_uart(void);

#endif /* UPLINK_OUTPUT_SOURCE_SELECTOR_H */
