#ifndef DOWNLINK_DATA_ROUTER_H
#define DOWNLINK_DATA_ROUTER_H

#include "stm32f4xx_hal.h"

void data_router_init(void);
void data_router_poll(void);
void data_router_on_uart_rx_complete(UART_HandleTypeDef *huart);
void data_router_on_uart_error(UART_HandleTypeDef *huart);
void data_router_trace_usart6_irq_enter(void);
void data_router_trace_dma2_stream1_irq_enter(void);
void data_router_trace_irq_exit(void);

#endif /* DOWNLINK_DATA_ROUTER_H */
