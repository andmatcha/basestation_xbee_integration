#include "modules/rgb_led_driver.h"

#include "main.h"

#include <stdbool.h>
#include <string.h>

#define RGB_LED_TIMER                    htim3
#define RGB_LED_MODE_CHANNEL             TIM_CHANNEL_3
#define RGB_LED_STATE_CHANNEL            TIM_CHANNEL_4
#define RGB_LED_COUNT                    1U
#define RGB_LED_BITS_PER_LED             24U
#define RGB_LED_RESET_SLOTS              60U
#define RGB_LED_BUFFER_SIZE              ((RGB_LED_COUNT * RGB_LED_BITS_PER_LED) + RGB_LED_RESET_SLOTS)
#define RGB_LED_TIMER_CLOCK_HZ           16000000U
#define RGB_LED_SIGNAL_FREQ_HZ           800000U
#define RGB_LED_PERIOD_TICKS             (RGB_LED_TIMER_CLOCK_HZ / RGB_LED_SIGNAL_FREQ_HZ)
#define RGB_LED_DUTY_0                   7U
#define RGB_LED_DUTY_1                   13U
#define RGB_LED_MAX_BRIGHTNESS_PERCENT   50U

typedef struct
{
    uint32_t timer_channel;
    rgb_led_color_t target_color;
    rgb_led_color_t active_color;
    rgb_led_color_t in_flight_color;
    uint16_t pwm_data[RGB_LED_BUFFER_SIZE];
} RgbLedOutputContext;

typedef struct
{
    bool busy;
    uint32_t active_channel;
    RgbLedOutputContext mode_led;
    RgbLedOutputContext state_led;
} RgbLedDriverContext;

static RgbLedDriverContext g_rgb_led_driver;

extern TIM_HandleTypeDef htim3;

static bool colors_equal(rgb_led_color_t left, rgb_led_color_t right)
{
    return (left.red == right.red) && (left.green == right.green) &&
           (left.blue == right.blue);
}

static uint8_t limit_brightness(uint8_t channel)
{
    return (uint8_t)(((uint16_t)channel * RGB_LED_MAX_BRIGHTNESS_PERCENT) / 100U);
}

static void ws2812_clear(uint16_t *pwm_data)
{
    memset(pwm_data, 0, sizeof(uint16_t) * RGB_LED_BUFFER_SIZE);
}

static void ws2812_set_pixel(uint16_t *pwm_data, uint16_t index,
                             rgb_led_color_t color)
{
    uint32_t color_data;
    uint32_t base_index;

    if (index >= RGB_LED_COUNT) {
        return;
    }

    color_data = ((uint32_t)limit_brightness(color.green) << 16) |
                 ((uint32_t)limit_brightness(color.red) << 8) |
                 (uint32_t)limit_brightness(color.blue);
    base_index = (uint32_t)index * RGB_LED_BITS_PER_LED;

    for (uint32_t bit = 0U; bit < RGB_LED_BITS_PER_LED; bit++) {
        pwm_data[base_index + bit] =
            ((color_data & (1UL << (23U - bit))) != 0U) ? RGB_LED_DUTY_1
                                                        : RGB_LED_DUTY_0;
    }
}

static void build_pwm_data(RgbLedOutputContext *output)
{
    ws2812_clear(output->pwm_data);
    ws2812_set_pixel(output->pwm_data, 0U, output->in_flight_color);
}

static bool output_has_pending_refresh(const RgbLedOutputContext *output)
{
    return !colors_equal(output->active_color, output->target_color);
}

static RgbLedOutputContext *get_output_by_channel(uint32_t timer_channel)
{
    if (timer_channel == g_rgb_led_driver.mode_led.timer_channel) {
        return &g_rgb_led_driver.mode_led;
    }

    if (timer_channel == g_rgb_led_driver.state_led.timer_channel) {
        return &g_rgb_led_driver.state_led;
    }

    return NULL;
}

static RgbLedOutputContext *get_next_output_to_refresh(void)
{
    if (output_has_pending_refresh(&g_rgb_led_driver.mode_led)) {
        return &g_rgb_led_driver.mode_led;
    }

    if (output_has_pending_refresh(&g_rgb_led_driver.state_led)) {
        return &g_rgb_led_driver.state_led;
    }

    return NULL;
}

static void start_output_transfer(RgbLedOutputContext *output)
{
    if (RGB_LED_PERIOD_TICKS == 0U) {
        Error_Handler();
    }

    output->in_flight_color = output->target_color;
    build_pwm_data(output);

    g_rgb_led_driver.busy = true;
    g_rgb_led_driver.active_channel = output->timer_channel;

    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, output->timer_channel, 0U);
    if (HAL_TIM_PWM_Start_DMA(&RGB_LED_TIMER, output->timer_channel,
                              (uint32_t *)output->pwm_data,
                              RGB_LED_BUFFER_SIZE) != HAL_OK) {
        g_rgb_led_driver.busy = false;
        g_rgb_led_driver.active_channel = 0U;
        Error_Handler();
    }
}

void rgb_led_driver_init(void)
{
    memset(&g_rgb_led_driver, 0, sizeof(g_rgb_led_driver));

    g_rgb_led_driver.mode_led.timer_channel = RGB_LED_MODE_CHANNEL;
    g_rgb_led_driver.state_led.timer_channel = RGB_LED_STATE_CHANNEL;

    ws2812_clear(g_rgb_led_driver.mode_led.pwm_data);
    ws2812_clear(g_rgb_led_driver.state_led.pwm_data);

    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_MODE_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_STATE_CHANNEL, 0U);
}

void rgb_led_driver_poll(void)
{
    RgbLedOutputContext *next_output;

    if (g_rgb_led_driver.busy) {
        return;
    }

    next_output = get_next_output_to_refresh();
    if (next_output == NULL) {
        return;
    }

    start_output_transfer(next_output);
}

void rgb_led_driver_set_colors(rgb_led_color_t mode_led,
                               rgb_led_color_t state_led)
{
    g_rgb_led_driver.mode_led.target_color = mode_led;
    g_rgb_led_driver.state_led.target_color = state_led;
}

void rgb_led_driver_on_pwm_pulse_finished(TIM_HandleTypeDef *htim)
{
    RgbLedOutputContext *output;

    if ((htim != &RGB_LED_TIMER) || !g_rgb_led_driver.busy) {
        return;
    }

    output = get_output_by_channel(g_rgb_led_driver.active_channel);
    if (output == NULL) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Stop_DMA(&RGB_LED_TIMER, g_rgb_led_driver.active_channel) != HAL_OK) {
        Error_Handler();
    }

    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, g_rgb_led_driver.active_channel, 0U);

    output->active_color = output->in_flight_color;
    g_rgb_led_driver.active_channel = 0U;
    g_rgb_led_driver.busy = false;
}
