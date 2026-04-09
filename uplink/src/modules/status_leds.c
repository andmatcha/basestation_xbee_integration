#include "modules/status_leds.h"

#include "modules/output_source_selector.h"
#include "modules/rgb_led_driver.h"

#include <stdbool.h>
#include <stdint.h>

#define MODE_LED_USB_RED                  255U
#define MODE_LED_USB_GREEN                96U
#define MODE_LED_USB_BLUE                 0U
#define MODE_LED_XBEE_RED                 0U
#define MODE_LED_XBEE_GREEN               160U
#define MODE_LED_XBEE_BLUE                255U

#define STATE_LED_IDLE_RED                0U
#define STATE_LED_IDLE_GREEN              255U
#define STATE_LED_IDLE_BLUE               0U

#define STATE_LED_ACTIVITY_LOW_BPS        50U
#define STATE_LED_ACTIVITY_MID_BPS        800U
#define STATE_LED_ACTIVITY_HIGH_BPS       3000U

#define STATE_LED_ACTIVITY_LOW_RED        0U
#define STATE_LED_ACTIVITY_LOW_GREEN      160U
#define STATE_LED_ACTIVITY_LOW_BLUE       255U
#define STATE_LED_ACTIVITY_MID_RED        255U
#define STATE_LED_ACTIVITY_MID_GREEN      220U
#define STATE_LED_ACTIVITY_MID_BLUE       0U
#define STATE_LED_ACTIVITY_HIGH_RED       255U
#define STATE_LED_ACTIVITY_HIGH_GREEN     32U
#define STATE_LED_ACTIVITY_HIGH_BLUE      0U

#define STATE_LED_RATE_WINDOW_MS          500U
#define STATE_LED_ACTIVITY_HOLD_MS        250U
#define STATE_LED_BLINK_HALF_PERIOD_MS    60U

typedef struct
{
    uint32_t rx_window_start_ms;
    uint32_t last_rx_tick_ms;
    uint32_t rx_bytes_in_window;
    bool has_seen_rx;
} StatusLedsContext;

static StatusLedsContext g_status_leds;

static rgb_led_color_t make_color(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb_led_color_t color = {red, green, blue};
    return color;
}

static uint8_t lerp_channel(uint8_t start, uint8_t end,
                            uint32_t position, uint32_t span)
{
    int32_t delta;

    if (span == 0U) {
        return end;
    }

    delta = (int32_t)end - (int32_t)start;
    return (uint8_t)((int32_t)start + ((delta * (int32_t)position) / (int32_t)span));
}

static rgb_led_color_t lerp_color(rgb_led_color_t start, rgb_led_color_t end,
                                  uint32_t position, uint32_t span)
{
    rgb_led_color_t color;

    color.red = lerp_channel(start.red, end.red, position, span);
    color.green = lerp_channel(start.green, end.green, position, span);
    color.blue = lerp_channel(start.blue, end.blue, position, span);
    return color;
}

static rgb_led_color_t get_mode_led_color(void)
{
    if (uplink_output_source_get_current() == UPLINK_OUTPUT_SOURCE_USB) {
        return make_color(MODE_LED_USB_RED, MODE_LED_USB_GREEN, MODE_LED_USB_BLUE);
    }

    return make_color(MODE_LED_XBEE_RED, MODE_LED_XBEE_GREEN, MODE_LED_XBEE_BLUE);
}

static rgb_led_color_t get_idle_state_led_color(void)
{
    return make_color(STATE_LED_IDLE_RED, STATE_LED_IDLE_GREEN,
                      STATE_LED_IDLE_BLUE);
}

static rgb_led_color_t get_activity_state_led_color(uint32_t rx_rate_bps)
{
    const rgb_led_color_t low =
        make_color(STATE_LED_ACTIVITY_LOW_RED, STATE_LED_ACTIVITY_LOW_GREEN,
                   STATE_LED_ACTIVITY_LOW_BLUE);
    const rgb_led_color_t mid =
        make_color(STATE_LED_ACTIVITY_MID_RED, STATE_LED_ACTIVITY_MID_GREEN,
                   STATE_LED_ACTIVITY_MID_BLUE);
    const rgb_led_color_t high =
        make_color(STATE_LED_ACTIVITY_HIGH_RED, STATE_LED_ACTIVITY_HIGH_GREEN,
                   STATE_LED_ACTIVITY_HIGH_BLUE);

    if (rx_rate_bps <= STATE_LED_ACTIVITY_LOW_BPS) {
        return low;
    }

    if (rx_rate_bps >= STATE_LED_ACTIVITY_HIGH_BPS) {
        return high;
    }

    if (rx_rate_bps <= STATE_LED_ACTIVITY_MID_BPS) {
        return lerp_color(low, mid,
                          rx_rate_bps - STATE_LED_ACTIVITY_LOW_BPS,
                          STATE_LED_ACTIVITY_MID_BPS - STATE_LED_ACTIVITY_LOW_BPS);
    }

    return lerp_color(mid, high,
                      rx_rate_bps - STATE_LED_ACTIVITY_MID_BPS,
                      STATE_LED_ACTIVITY_HIGH_BPS - STATE_LED_ACTIVITY_MID_BPS);
}

static uint32_t get_recent_rx_rate_bps(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - g_status_leds.rx_window_start_ms;

    if (!g_status_leds.has_seen_rx) {
        return 0U;
    }

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }

    if (elapsed_ms > STATE_LED_RATE_WINDOW_MS) {
        elapsed_ms = STATE_LED_RATE_WINDOW_MS;
    }

    return (g_status_leds.rx_bytes_in_window * 1000U) / elapsed_ms;
}

static rgb_led_color_t get_state_led_color(uint32_t now_ms)
{
    bool is_receiving;
    bool blink_on;

    if (!g_status_leds.has_seen_rx) {
        return get_idle_state_led_color();
    }

    is_receiving =
        (now_ms - g_status_leds.last_rx_tick_ms) <= STATE_LED_ACTIVITY_HOLD_MS;
    if (!is_receiving) {
        return get_idle_state_led_color();
    }

    blink_on = ((now_ms / STATE_LED_BLINK_HALF_PERIOD_MS) & 0x1U) == 0U;
    if (!blink_on) {
        return make_color(0U, 0U, 0U);
    }

    return get_activity_state_led_color(get_recent_rx_rate_bps(now_ms));
}

void status_leds_init(void)
{
    g_status_leds.rx_window_start_ms = HAL_GetTick();
    g_status_leds.last_rx_tick_ms = 0U;
    g_status_leds.rx_bytes_in_window = 0U;
    g_status_leds.has_seen_rx = false;

    rgb_led_driver_init();
}

void status_leds_poll(void)
{
    rgb_led_color_t mode_led;
    rgb_led_color_t state_led;
    uint32_t now_ms = HAL_GetTick();

    mode_led = get_mode_led_color();
    state_led = get_state_led_color(now_ms);

    rgb_led_driver_set_colors(mode_led, state_led);
    rgb_led_driver_poll();
}

void status_leds_on_rx_activity(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (!g_status_leds.has_seen_rx ||
        ((now_ms - g_status_leds.last_rx_tick_ms) > STATE_LED_RATE_WINDOW_MS) ||
        ((now_ms - g_status_leds.rx_window_start_ms) > STATE_LED_RATE_WINDOW_MS)) {
        g_status_leds.rx_window_start_ms = now_ms;
        g_status_leds.rx_bytes_in_window = 0U;
    }

    g_status_leds.last_rx_tick_ms = now_ms;
    g_status_leds.rx_bytes_in_window++;
    g_status_leds.has_seen_rx = true;
}

void status_leds_on_tim_pwm_pulse_finished(TIM_HandleTypeDef *htim)
{
    rgb_led_driver_on_pwm_pulse_finished(htim);
}
