#include "modules/input_source_selector.h"

#include "debug_log.h"
#include "main.h"
#include "modules/buzzer.h"

#define DOWNLINK_INPUT_SWITCH_DEBOUNCE_MS 30U
#define DOWNLINK_INPUT_SHORT_PRESS_MAX_MS 350U
#define DOWNLINK_INPUT_DOUBLE_PUSH_GAP_MS 400U

extern UART_HandleTypeDef huart4;

typedef struct
{
    GPIO_PinState last_sampled_level;
    GPIO_PinState stable_level;
    uint32_t last_transition_tick_ms;
    uint32_t pressed_tick_ms;
    uint32_t last_short_release_tick_ms;
    bool science_mode_enabled;
    uint8_t short_press_count;
} DownlinkInputSourceContext;

static DownlinkInputSourceContext g_input_source_context;

static GPIO_PinState read_switch_level(void)
{
    return HAL_GPIO_ReadPin(Push_Switch_GPIO_Port, Push_Switch_Pin);
}

static bool has_elapsed(uint32_t start_tick_ms, uint32_t duration_ms)
{
    return (HAL_GetTick() - start_tick_ms) >= duration_ms;
}

static void set_science_mode_enabled(bool enabled)
{
    if (g_input_source_context.science_mode_enabled == enabled) {
        return;
    }

    g_input_source_context.science_mode_enabled = enabled;
    LOG("[downlink] science mode -> %s\r\n", enabled ? "enabled" : "disabled");
    buzzer_play_mode_switch_melody();
}

static void clear_short_press_sequence(void)
{
    g_input_source_context.short_press_count = 0U;
    g_input_source_context.last_short_release_tick_ms = 0U;
}

static void register_short_press(uint32_t now_ms)
{
    if ((g_input_source_context.short_press_count == 0U) ||
        has_elapsed(g_input_source_context.last_short_release_tick_ms,
                    DOWNLINK_INPUT_DOUBLE_PUSH_GAP_MS)) {
        g_input_source_context.short_press_count = 1U;
        g_input_source_context.last_short_release_tick_ms = now_ms;
        return;
    }

    g_input_source_context.short_press_count++;
    g_input_source_context.last_short_release_tick_ms = now_ms;

    if (g_input_source_context.short_press_count >= 2U) {
        set_science_mode_enabled(!g_input_source_context.science_mode_enabled);
        clear_short_press_sequence();
    }
}

static void handle_stable_switch_transition(GPIO_PinState stable_level, uint32_t now_ms)
{
    uint32_t press_duration_ms = 0U;

    g_input_source_context.stable_level = stable_level;

    if (stable_level == GPIO_PIN_RESET) {
        g_input_source_context.pressed_tick_ms = now_ms;
        return;
    }

    if (g_input_source_context.pressed_tick_ms != 0U) {
        press_duration_ms = now_ms - g_input_source_context.pressed_tick_ms;
    }

    if ((press_duration_ms > 0U) &&
        (press_duration_ms <= DOWNLINK_INPUT_SHORT_PRESS_MAX_MS)) {
        register_short_press(now_ms);
    } else {
        clear_short_press_sequence();
    }

    g_input_source_context.pressed_tick_ms = 0U;
}

void input_source_selector_init(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState switch_level = read_switch_level();

    g_input_source_context.last_sampled_level = switch_level;
    g_input_source_context.stable_level = switch_level;
    g_input_source_context.last_transition_tick_ms = now_ms;
    g_input_source_context.pressed_tick_ms =
        (switch_level == GPIO_PIN_RESET) ? now_ms : 0U;
    g_input_source_context.last_short_release_tick_ms = 0U;
    g_input_source_context.science_mode_enabled = false;
    g_input_source_context.short_press_count = 0U;

    LOG("[downlink] input source -> %s\r\n",
        downlink_input_source_get_current_name());
}

void input_source_selector_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState sampled_level = read_switch_level();

    if (sampled_level != g_input_source_context.last_sampled_level) {
        g_input_source_context.last_sampled_level = sampled_level;
        g_input_source_context.last_transition_tick_ms = now_ms;
    }

    if ((g_input_source_context.stable_level != g_input_source_context.last_sampled_level) &&
        has_elapsed(g_input_source_context.last_transition_tick_ms,
                    DOWNLINK_INPUT_SWITCH_DEBOUNCE_MS)) {
        handle_stable_switch_transition(g_input_source_context.last_sampled_level, now_ms);
    }

    if ((g_input_source_context.short_press_count > 0U) &&
        has_elapsed(g_input_source_context.last_short_release_tick_ms,
                    DOWNLINK_INPUT_DOUBLE_PUSH_GAP_MS)) {
        clear_short_press_sequence();
    }
}

DownlinkInputSource downlink_input_source_get_current(void)
{
    return DOWNLINK_INPUT_SOURCE_LINK;
}

const char *downlink_input_source_get_current_name(void)
{
    return "UART4 IN";
}

UART_HandleTypeDef *downlink_input_source_get_active_uart(void)
{
    return &huart4;
}

bool downlink_input_source_is_selected_uart(const UART_HandleTypeDef *huart)
{
    return huart == downlink_input_source_get_active_uart();
}

bool downlink_input_source_is_science_mode_enabled(void)
{
    return g_input_source_context.science_mode_enabled;
}
