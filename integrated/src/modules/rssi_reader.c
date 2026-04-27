#include "modules/rssi_reader.h"

#include "debug_log.h"
#include "main.h"
#include "modules/display_manager.h"
#include "modules/mode_control.h"

#define RSSI_LOG_INTERVAL_MS 1000U

typedef struct
{
    uint32_t last_log_ms;
    bool started;
} rssi_reader_context_t;

static rssi_reader_context_t g_rssi;

extern TIM_HandleTypeDef htim2;

void rssi_reader_init(void)
{
    g_rssi.last_log_ms = HAL_GetTick();
    g_rssi.started = false;

    if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        LOG("[integrated] rssi timer ch1 start failed\r\n");
        display_manager_show_error();
        return;
    }

    if (HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        LOG("[integrated] rssi timer ch2 start failed\r\n");
        display_manager_show_error();
        return;
    }

    g_rssi.started = true;
}

void rssi_reader_poll(void)
{
#if DEBUG_LOG_ENABLED
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t period_us = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    const uint32_t high_us = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);
    uint32_t duty_permille = 0U;

    if (!g_rssi.started ||
        (mode_control_get_xbee_mode() != XBEE_MODE_ONBOARD) ||
        ((now_ms - g_rssi.last_log_ms) < RSSI_LOG_INTERVAL_MS)) {
        return;
    }

    g_rssi.last_log_ms = now_ms;

    if (period_us != 0U) {
        duty_permille = (high_us * 1000U) / period_us;
    }

    LOG("[integrated] rssi pwm high=%luus period=%luus duty=%lu.%lu%%\r\n",
        (unsigned long)high_us,
        (unsigned long)period_us,
        (unsigned long)(duty_permille / 10U),
        (unsigned long)(duty_permille % 10U));
#endif
}
