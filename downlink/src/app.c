#include "app.h"

#include "modules/data_router.h"
#include "modules/input_source_selector.h"

void init(void)
{
    input_source_selector_init();
    data_router_init();
}

void poll(void)
{
    input_source_selector_poll();
    data_router_poll();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    data_router_on_uart_rx_complete(huart);
}
