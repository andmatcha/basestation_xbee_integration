#include "modules/display_manager.h"

#include "debug_log.h"
#include "main.h"
#include "modules/buzzer.h"
#include "modules/display_view.h"
#include "modules/lcd_driver.h"

#include <stdbool.h>
#include <string.h>

#define DISPLAY_SWITCH_DEBOUNCE_MS  30U
#define DISPLAY_SWITCH_LONG_PRESS_MS  1000U
#define DISPLAY_REFRESH_MS          200U
#define DISPLAY_STARTUP_MS          1000U

typedef struct
{
    display_view_mode_t mode;
    GPIO_PinState last_sampled_level;
    GPIO_PinState stable_level;
    uint32_t last_transition_ms;
    uint32_t pressed_ms;
    uint32_t startup_until_ms;
    uint32_t last_refresh_ms;
    bool long_press_handled;
    bool error;
    bool initialized;
} display_manager_context_t;

static display_manager_context_t g_display;

static GPIO_PinState read_display_switch(void)
{
    return HAL_GPIO_ReadPin(PUSH_SWITCH_2_GPIO_Port, PUSH_SWITCH_2_Pin);
}

static bool has_elapsed(uint32_t start_ms, uint32_t duration_ms)
{
    return (HAL_GetTick() - start_ms) >= duration_ms;
}

static void write_startup_display(void)
{
    char line0[DISPLAY_VIEW_LINE_LEN + 1U];
    char line1[DISPLAY_VIEW_LINE_LEN + 1U];

    display_view_build_startup(line0, line1);
    (void)lcd_driver_write_lines(line0, line1);
}

static void refresh_display(bool force)
{
    char line0[DISPLAY_VIEW_LINE_LEN + 1U];
    char line1[DISPLAY_VIEW_LINE_LEN + 1U];
    const uint32_t now_ms = HAL_GetTick();

    if (!g_display.initialized) {
        return;
    }

    if (!force && !has_elapsed(g_display.last_refresh_ms, DISPLAY_REFRESH_MS)) {
        return;
    }

    if (g_display.error) {
        display_view_build_error(line0, line1);
    } else {
        display_view_build_mode(g_display.mode, line0, line1);
    }

    if (lcd_driver_write_lines(line0, line1) != HAL_OK) {
        LOG("[integrated] lcd refresh failed\r\n");
        g_display.error = true;
    }

    g_display.last_refresh_ms = now_ms;
}

static void toggle_display_mode(void)
{
    g_display.mode =
        (g_display.mode == DISPLAY_VIEW_MODE_STATUS) ? DISPLAY_VIEW_MODE_RATE
                                                     : DISPLAY_VIEW_MODE_STATUS;
    LOG("[integrated] display mode -> %s\r\n",
        display_view_mode_name(g_display.mode));
    buzzer_play_short_beep();
    refresh_display(true);
}

static void toggle_downlink_raw_log(void)
{
    debug_log_set_downlink_raw_mode(!debug_log_is_downlink_raw_mode());
    buzzer_play_double_beep();
    refresh_display(true);
}

static void handle_stable_transition(GPIO_PinState stable_level, uint32_t now_ms)
{
    g_display.stable_level = stable_level;

    if (stable_level == GPIO_PIN_RESET) {
        g_display.pressed_ms = now_ms;
        g_display.long_press_handled = false;
        return;
    }

    if (!g_display.long_press_handled) {
        toggle_display_mode();
    }

    g_display.pressed_ms = 0U;
    g_display.long_press_handled = false;
}

void display_manager_init(I2C_HandleTypeDef *hi2c)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState level = read_display_switch();

    memset(&g_display, 0, sizeof(g_display));
    g_display.mode = DISPLAY_VIEW_MODE_STATUS;
    g_display.last_sampled_level = level;
    g_display.stable_level = level;
    g_display.last_transition_ms = now_ms;
    g_display.pressed_ms = (level == GPIO_PIN_RESET) ? now_ms : 0U;
    g_display.startup_until_ms = now_ms + DISPLAY_STARTUP_MS;
    g_display.last_refresh_ms = 0U;
    g_display.long_press_handled = false;

    if (lcd_driver_init(hi2c, LCD_RESET_GPIO_Port, LCD_RESET_Pin) != HAL_OK) {
        LOG("[integrated] lcd init failed\r\n");
        g_display.error = true;
        return;
    }

    g_display.initialized = true;
    write_startup_display();
}

void display_manager_wait_startup_done(void)
{
    const uint32_t now_ms = HAL_GetTick();

    if ((int32_t)(g_display.startup_until_ms - now_ms) > 0) {
        HAL_Delay(g_display.startup_until_ms - now_ms);
    }

    refresh_display(true);
}

void display_manager_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState sampled_level = read_display_switch();

    if (sampled_level != g_display.last_sampled_level) {
        g_display.last_sampled_level = sampled_level;
        g_display.last_transition_ms = now_ms;
    }

    if ((g_display.stable_level != g_display.last_sampled_level) &&
        has_elapsed(g_display.last_transition_ms, DISPLAY_SWITCH_DEBOUNCE_MS)) {
        handle_stable_transition(g_display.last_sampled_level, now_ms);
    }

    if ((g_display.stable_level == GPIO_PIN_RESET) &&
        !g_display.long_press_handled &&
        has_elapsed(g_display.pressed_ms, DISPLAY_SWITCH_LONG_PRESS_MS)) {
        toggle_downlink_raw_log();
        g_display.long_press_handled = true;
    }

    refresh_display(false);
}

void display_manager_show_error(void)
{
    g_display.error = true;
    refresh_display(true);
}
