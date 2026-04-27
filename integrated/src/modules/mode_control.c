#include "modules/mode_control.h"

#include "debug_log.h"
#include "main.h"
#include "modules/buzzer.h"

#define MODE_SWITCH_DEBOUNCE_MS      30U
#define MODE_SWITCH_DOUBLE_CLICK_MS  350U

typedef struct
{
    module_mode_t module_mode;
    xbee_mode_t xbee_mode;
    GPIO_PinState last_sampled_level;
    GPIO_PinState stable_level;
    uint32_t last_transition_ms;
    uint32_t pressed_ms;
    uint32_t first_release_ms;
    uint32_t generation;
    bool single_click_pending;
} mode_control_context_t;

static mode_control_context_t g_mode;

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

static GPIO_PinState read_mode_switch(void)
{
    return HAL_GPIO_ReadPin(Push_Switch_1_GPIO_Port, Push_Switch_1_Pin);
}

static bool has_elapsed(uint32_t start_ms, uint32_t duration_ms)
{
    return (HAL_GetTick() - start_ms) >= duration_ms;
}

static void toggle_module_mode(void)
{
    g_mode.module_mode =
        (g_mode.module_mode == MODULE_MODE_ARM) ? MODULE_MODE_SCIENCE
                                                  : MODULE_MODE_ARM;
    g_mode.generation++;
    LOG("[integrated] module mode -> %s\r\n", mode_control_get_module_name());
    buzzer_play_short_beep();
}

static void toggle_xbee_mode(void)
{
    g_mode.xbee_mode =
        (g_mode.xbee_mode == XBEE_MODE_ONBOARD) ? XBEE_MODE_EXTERNAL
                                                : XBEE_MODE_ONBOARD;
    g_mode.generation++;
    LOG("[integrated] xbee mode -> %s\r\n", mode_control_get_xbee_name());
    buzzer_play_double_beep();
}

static void handle_release(uint32_t now_ms)
{
    g_mode.pressed_ms = 0U;

    if (g_mode.single_click_pending &&
        !has_elapsed(g_mode.first_release_ms, MODE_SWITCH_DOUBLE_CLICK_MS)) {
        g_mode.single_click_pending = false;
        toggle_xbee_mode();
        return;
    }

    g_mode.single_click_pending = true;
    g_mode.first_release_ms = now_ms;
}

static void handle_stable_transition(GPIO_PinState stable_level, uint32_t now_ms)
{
    g_mode.stable_level = stable_level;

    if (stable_level == GPIO_PIN_RESET) {
        g_mode.pressed_ms = now_ms;
        return;
    }

    handle_release(now_ms);
}

void mode_control_init(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState level = read_mode_switch();

    g_mode.module_mode = MODULE_MODE_ARM;
    g_mode.xbee_mode = XBEE_MODE_ONBOARD;
    g_mode.last_sampled_level = level;
    g_mode.stable_level = level;
    g_mode.last_transition_ms = now_ms;
    g_mode.pressed_ms = (level == GPIO_PIN_RESET) ? now_ms : 0U;
    g_mode.first_release_ms = 0U;
    g_mode.generation = 0U;
    g_mode.single_click_pending = false;

    LOG("[integrated] default mode -> %s / %s\r\n",
        mode_control_get_module_name(), mode_control_get_xbee_name());
}

void mode_control_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState sampled_level = read_mode_switch();

    if (sampled_level != g_mode.last_sampled_level) {
        g_mode.last_sampled_level = sampled_level;
        g_mode.last_transition_ms = now_ms;
    }

    if ((g_mode.stable_level != g_mode.last_sampled_level) &&
        has_elapsed(g_mode.last_transition_ms, MODE_SWITCH_DEBOUNCE_MS)) {
        handle_stable_transition(g_mode.last_sampled_level, now_ms);
    }

    if (g_mode.single_click_pending &&
        has_elapsed(g_mode.first_release_ms, MODE_SWITCH_DOUBLE_CLICK_MS)) {
        g_mode.single_click_pending = false;
        toggle_module_mode();
    }
}

module_mode_t mode_control_get_module_mode(void)
{
    return g_mode.module_mode;
}

xbee_mode_t mode_control_get_xbee_mode(void)
{
    return g_mode.xbee_mode;
}

uint32_t mode_control_get_generation(void)
{
    return g_mode.generation;
}

const char *mode_control_get_module_name(void)
{
    return (g_mode.module_mode == MODULE_MODE_ARM) ? "Arm" : "Science";
}

const char *mode_control_get_xbee_name(void)
{
    return (g_mode.xbee_mode == XBEE_MODE_ONBOARD) ? "Onboard" : "External";
}

UART_HandleTypeDef *mode_control_get_active_xbee_uart(void)
{
    return (g_mode.xbee_mode == XBEE_MODE_ONBOARD) ? &huart6 : &huart3;
}

bool mode_control_is_active_xbee_uart(const UART_HandleTypeDef *huart)
{
    return huart == mode_control_get_active_xbee_uart();
}
