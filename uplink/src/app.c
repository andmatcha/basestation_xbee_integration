#include "app.h"

#include "modules/data_router.h"
#include "modules/output_source_selector.h"
#include "modules/status_leds.h"

void init(void)
{
    output_source_selector_init();
    status_leds_init();
    data_router_init();
    status_leds_poll();
}

void poll(void)
{
    output_source_selector_poll();
    data_router_poll();
    status_leds_poll();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    data_router_on_uart_rx_complete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    data_router_on_uart_tx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    data_router_on_uart_error(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    status_leds_on_tim_period_elapsed(htim);
}
