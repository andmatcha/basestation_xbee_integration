#include "modules/display_manager.h"

#include "debug_log.h"
#include "main.h"
#include "modules/buzzer.h"
#include "modules/lcd_driver.h"
#include "modules/link_stats.h"
#include "modules/mode_control.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define DISPLAY_SWITCH_DEBOUNCE_MS  30U
#define DISPLAY_REFRESH_MS          200U
#define DISPLAY_STARTUP_MS          1000U
#define DISPLAY_LINE_LEN            16U

typedef enum
{
    DISPLAY_MODE_STATUS = 0,
    DISPLAY_MODE_RATE,
} display_mode_t;

typedef struct
{
    display_mode_t mode;
    GPIO_PinState last_sampled_level;
    GPIO_PinState stable_level;
    uint32_t last_transition_ms;
    uint32_t startup_until_ms;
    uint32_t last_refresh_ms;
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

static void pad_line(char *line)
{
    size_t len = strlen(line);

    if (len >= DISPLAY_LINE_LEN) {
        line[DISPLAY_LINE_LEN] = '\0';
        return;
    }

    memset(&line[len], ' ', DISPLAY_LINE_LEN - len);
    line[DISPLAY_LINE_LEN] = '\0';
}

static uint32_t clamp_rate_999(uint32_t rate)
{
    return (rate > 999U) ? 999U : rate;
}

static uint32_t clamp_rate_99(uint32_t rate)
{
    return (rate > 99U) ? 99U : rate;
}

static void format_rf_rate_part(char *buffer, size_t size,
                                const char *label, uint32_t rate)
{
    rate = clamp_rate_999(rate);

    if (rate >= 100U) {
        (void)snprintf(buffer, size, "%s%03lu", label, (unsigned long)rate);
        return;
    }

    (void)snprintf(buffer, size, "%s%luHz", label, (unsigned long)rate);
}

static void build_status_lines(char *line0, char *line1)
{
    (void)snprintf(line0, DISPLAY_LINE_LEN + 1U, "%s %s",
                   mode_control_get_module_name(),
                   mode_control_get_xbee_name());
    pad_line(line0);

    (void)snprintf(line1, DISPLAY_LINE_LEN + 1U, "UP:%s DOWN:%s",
                   link_stats_get_status_code(LINK_STAT_UPLINK),
                   link_stats_get_status_code(LINK_STAT_DOWNLINK));
    pad_line(line1);
}

static void build_rate_lines(char *line0, char *line1)
{
    const link_stat_snapshot_t rf = link_stats_get_snapshot(LINK_STAT_RF);
    const link_stat_snapshot_t uplink = link_stats_get_snapshot(LINK_STAT_UPLINK);
    const link_stat_snapshot_t downlink = link_stats_get_snapshot(LINK_STAT_DOWNLINK);
    char tx_part[8];
    char rx_part[8];

    format_rf_rate_part(tx_part, sizeof(tx_part), "TX", rf.tx_hz);
    format_rf_rate_part(rx_part, sizeof(rx_part), "RX", rf.rx_hz);
    (void)snprintf(line0, DISPLAY_LINE_LEN + 1U, "RF:%s/%s", tx_part, rx_part);
    pad_line(line0);

    (void)snprintf(line1, DISPLAY_LINE_LEN + 1U, "U:%lu/%lu D:%lu/%lu",
                   (unsigned long)clamp_rate_99(uplink.tx_hz),
                   (unsigned long)clamp_rate_99(uplink.rx_hz),
                   (unsigned long)clamp_rate_99(downlink.tx_hz),
                   (unsigned long)clamp_rate_99(downlink.rx_hz));
    pad_line(line1);
}

static void refresh_display(bool force)
{
    char line0[DISPLAY_LINE_LEN + 1U];
    char line1[DISPLAY_LINE_LEN + 1U];
    const uint32_t now_ms = HAL_GetTick();

    if (!g_display.initialized) {
        return;
    }

    if (!force && !has_elapsed(g_display.last_refresh_ms, DISPLAY_REFRESH_MS)) {
        return;
    }

    if (g_display.error) {
        (void)snprintf(line0, sizeof(line0), "ERROR OCCURED");
        line1[0] = '\0';
        pad_line(line0);
        pad_line(line1);
    } else if (g_display.mode == DISPLAY_MODE_RATE) {
        build_rate_lines(line0, line1);
    } else {
        build_status_lines(line0, line1);
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
        (g_display.mode == DISPLAY_MODE_STATUS) ? DISPLAY_MODE_RATE
                                                : DISPLAY_MODE_STATUS;
    LOG("[integrated] display mode -> %s\r\n",
        (g_display.mode == DISPLAY_MODE_STATUS) ? "status" : "rate");
    buzzer_play_short_beep();
    refresh_display(true);
}

static void handle_stable_transition(GPIO_PinState stable_level)
{
    g_display.stable_level = stable_level;

    if (stable_level == GPIO_PIN_SET) {
        toggle_display_mode();
    }
}

void display_manager_init(I2C_HandleTypeDef *hi2c)
{
    const uint32_t now_ms = HAL_GetTick();
    const GPIO_PinState level = read_display_switch();

    memset(&g_display, 0, sizeof(g_display));
    g_display.mode = DISPLAY_MODE_STATUS;
    g_display.last_sampled_level = level;
    g_display.stable_level = level;
    g_display.last_transition_ms = now_ms;
    g_display.startup_until_ms = now_ms + DISPLAY_STARTUP_MS;
    g_display.last_refresh_ms = 0U;

    if (lcd_driver_init(hi2c, LCD_RESET_GPIO_Port, LCD_RESET_Pin) != HAL_OK) {
        LOG("[integrated] lcd init failed\r\n");
        g_display.error = true;
        return;
    }

    g_display.initialized = true;
    (void)lcd_driver_write_lines("KONNICHIWA", "");
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
        handle_stable_transition(g_display.last_sampled_level);
    }

    refresh_display(false);
}

void display_manager_show_error(void)
{
    g_display.error = true;
    refresh_display(true);
}
