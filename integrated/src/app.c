#include "app.h"

#include "debug_log.h"
#include "main.h"
#include "modules/buzzer.h"
#include "modules/data_router.h"
#include "modules/display_manager.h"
#include "modules/link_stats.h"
#include "modules/mode_control.h"
#include "modules/rssi_reader.h"

extern I2C_HandleTypeDef hi2c1;

void init(void)
{
    link_stats_init();
    mode_control_init();
    buzzer_init();
    display_manager_init(&hi2c1);
    buzzer_play_startup_beeps();
    display_manager_wait_startup_done();
    data_router_init();
    rssi_reader_init();
    LOG("[integrated] app initialized\r\n");
}

void poll(void)
{
    mode_control_poll();
    data_router_poll();
    link_stats_poll();
    rssi_reader_poll();
    display_manager_poll();
}

void app_on_error(void)
{
    LOG("[integrated] fatal error\r\n");
    display_manager_show_error();
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
