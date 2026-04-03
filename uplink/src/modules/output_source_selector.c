#include "modules/output_source_selector.h"

#include "debug_log.h"
#include "main.h"

#include <stdbool.h>

#define UPLINK_OUTPUT_SWITCH_DEBOUNCE_MS 30U
#define UPLINK_OUTPUT_SWITCH_HOLD_MS     2000U

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

typedef struct
{
    UplinkOutputSource active_source;
    GPIO_PinState last_sampled_level;
    GPIO_PinState stable_level;
    uint32_t last_transition_tick_ms;
    uint32_t pressed_tick_ms;
    bool long_press_handled;
} UplinkOutputSourceContext;

static UplinkOutputSourceContext g_output_source_context;

static GPIO_PinState read_switch_level(void)
{
    return HAL_GPIO_ReadPin(Push_Switch_GPIO_Port, Push_Switch_Pin);
}

static bool has_elapsed(uint32_t start_tick_ms, uint32_t duration_ms)
{
    return (HAL_GetTick() - start_tick_ms) >= duration_ms;
}

static void set_active_source(UplinkOutputSource source)
{
    g_output_source_context.active_source = source;
    LOG("[uplink] output source -> %s\r\n", uplink_output_source_get_current_name());
}

static void handle_stable_switch_transition(GPIO_PinState stable_level, uint32_t now_ms)
{
    g_output_source_context.stable_level = stable_level;

    if (stable_level == GPIO_PIN_RESET) {
        g_output_source_context.pressed_tick_ms = now_ms;
        g_output_source_context.long_press_handled = false;
        return;
    }

    g_output_source_context.pressed_tick_ms = 0U;
    g_output_source_context.long_press_handled = false;
}

void output_source_selector_init(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState switch_level = read_switch_level();

    g_output_source_context.active_source = UPLINK_OUTPUT_SOURCE_USB;
    g_output_source_context.last_sampled_level = switch_level;
    g_output_source_context.stable_level = switch_level;
    g_output_source_context.last_transition_tick_ms = now_ms;
    g_output_source_context.pressed_tick_ms =
        (switch_level == GPIO_PIN_RESET) ? now_ms : 0U;
    g_output_source_context.long_press_handled = false;

    LOG("[uplink] output source default -> %s\r\n",
        uplink_output_source_get_current_name());
}

void output_source_selector_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState sampled_level = read_switch_level();

    if (sampled_level != g_output_source_context.last_sampled_level) {
        g_output_source_context.last_sampled_level = sampled_level;
        g_output_source_context.last_transition_tick_ms = now_ms;
    }

    if ((g_output_source_context.stable_level != g_output_source_context.last_sampled_level) &&
        has_elapsed(g_output_source_context.last_transition_tick_ms,
                    UPLINK_OUTPUT_SWITCH_DEBOUNCE_MS)) {
        handle_stable_switch_transition(g_output_source_context.last_sampled_level, now_ms);
    }

    if ((g_output_source_context.stable_level == GPIO_PIN_RESET) &&
        !g_output_source_context.long_press_handled &&
        has_elapsed(g_output_source_context.pressed_tick_ms,
                    UPLINK_OUTPUT_SWITCH_HOLD_MS)) {
        set_active_source(
            (g_output_source_context.active_source == UPLINK_OUTPUT_SOURCE_USB)
                ? UPLINK_OUTPUT_SOURCE_XBEE
                : UPLINK_OUTPUT_SOURCE_USB);
        g_output_source_context.long_press_handled = true;
    }
}

UplinkOutputSource uplink_output_source_get_current(void)
{
    return g_output_source_context.active_source;
}

const char *uplink_output_source_get_current_name(void)
{
    return (g_output_source_context.active_source == UPLINK_OUTPUT_SOURCE_USB)
               ? "USB OUT (USART3)"
               : "XBee OUT (USART6)";
}

UART_HandleTypeDef *uplink_output_source_get_active_uart(void)
{
    return (g_output_source_context.active_source == UPLINK_OUTPUT_SOURCE_USB)
               ? &huart3
               : &huart6;
}
