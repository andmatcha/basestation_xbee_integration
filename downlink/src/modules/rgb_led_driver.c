#include "modules/rgb_led_driver.h"

#include "main.h"

#include <stdbool.h>
#include <string.h>

#define RGB_LED_TIMER                        htim3
#define RGB_LED_MODE_CHANNEL                 TIM_CHANNEL_3
#define RGB_LED_STATE_CHANNEL                TIM_CHANNEL_4
#define RGB_LED_DMA_REQUEST                  TIM_DMA_UPDATE
#define RGB_LED_DUTY_TICKS_0                 25U
#define RGB_LED_DUTY_TICKS_1                 58U
#define RGB_LED_BITS_PER_FRAME               24U
#define RGB_LED_RESET_SLOTS                  240U
#define RGB_LED_FRAME_SLOTS                  (RGB_LED_BITS_PER_FRAME + RGB_LED_RESET_SLOTS)
#define RGB_LED_CHANNELS_PER_SLOT            2U
#define RGB_LED_DMA_WORDS_PER_SLOT           RGB_LED_CHANNELS_PER_SLOT
#define RGB_LED_DMA_WORD_COUNT               (RGB_LED_FRAME_SLOTS * RGB_LED_DMA_WORDS_PER_SLOT)
#define RGB_LED_DMA_BURST_BASE               TIM_DMABASE_CCR3
#define RGB_LED_DMA_BURST_LENGTH             TIM_DMABURSTLENGTH_2TRANSFERS
#define RGB_LED_FIRST_DMA_SLOT_INDEX         2U

typedef struct
{
    rgb_led_color_t desired_mode_led;
    rgb_led_color_t desired_state_led;
    bool refresh_pending;
    bool frame_active;
    uint16_t dma_frame[RGB_LED_DMA_WORD_COUNT];
} RgbLedDriverContext;

static RgbLedDriverContext g_rgb_led_driver;

extern TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_up;

static bool colors_equal(rgb_led_color_t left, rgb_led_color_t right)
{
    return (left.red == right.red) && (left.green == right.green) &&
           (left.blue == right.blue);
}

static uint16_t get_duty_for_bit(bool is_set)
{
    return is_set ? RGB_LED_DUTY_TICKS_1 : RGB_LED_DUTY_TICKS_0;
}

static void encode_color_to_slot_range(rgb_led_color_t color, uint16_t slot_offset,
                                       bool is_mode_led)
{
    const uint8_t grb[3] = {color.green, color.red, color.blue};
    uint16_t slot = slot_offset;

    for (uint32_t byte_index = 0U; byte_index < 3U; byte_index++) {
        for (uint32_t bit_index = 0U; bit_index < 8U; bit_index++) {
            const bool is_set = (grb[byte_index] & (0x80U >> bit_index)) != 0U;
            const uint16_t duty = get_duty_for_bit(is_set);
            const uint16_t word_index = slot * RGB_LED_DMA_WORDS_PER_SLOT;

            if (is_mode_led) {
                g_rgb_led_driver.dma_frame[word_index] = duty;
            } else {
                g_rgb_led_driver.dma_frame[word_index + 1U] = duty;
            }

            slot++;
        }
    }
}

static void build_dma_frame(void)
{
    memset(g_rgb_led_driver.dma_frame, 0, sizeof(g_rgb_led_driver.dma_frame));
    encode_color_to_slot_range(g_rgb_led_driver.desired_mode_led, 0U, true);
    encode_color_to_slot_range(g_rgb_led_driver.desired_state_led, 0U, false);
}

static void apply_slot(uint16_t slot)
{
    const uint16_t word_index = slot * RGB_LED_DMA_WORDS_PER_SLOT;

    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_MODE_CHANNEL,
                          g_rgb_led_driver.dma_frame[word_index]);
    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_STATE_CHANNEL,
                          g_rgb_led_driver.dma_frame[word_index + 1U]);
}

static void stop_dma_frame(void)
{
    if (HAL_TIM_DMABurst_WriteStop(&RGB_LED_TIMER, RGB_LED_DMA_REQUEST) != HAL_OK) {
        Error_Handler();
    }
}

static void begin_dma_frame(void)
{
    HAL_StatusTypeDef status;
    const uint32_t dma_word_count =
        (RGB_LED_FRAME_SLOTS - RGB_LED_FIRST_DMA_SLOT_INDEX) * RGB_LED_DMA_WORDS_PER_SLOT;

    build_dma_frame();
    g_rgb_led_driver.frame_active = true;
    g_rgb_led_driver.refresh_pending = false;

    /* Prime slot 0 as the current PWM period and slot 1 as the first preload value.
     * DMA then supplies slot 2 onward on each update event. */
    apply_slot(0U);
    __HAL_TIM_SET_COUNTER(&RGB_LED_TIMER, 0U);
    if (HAL_TIM_GenerateEvent(&RGB_LED_TIMER, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        Error_Handler();
    }

    apply_slot(1U);

    status = HAL_TIM_DMABurst_MultiWriteStart(
        &RGB_LED_TIMER, RGB_LED_DMA_BURST_BASE, RGB_LED_DMA_REQUEST,
        (const uint32_t *)&g_rgb_led_driver.dma_frame[RGB_LED_FIRST_DMA_SLOT_INDEX *
                                                      RGB_LED_DMA_WORDS_PER_SLOT],
        RGB_LED_DMA_BURST_LENGTH, dma_word_count);
    if (status != HAL_OK) {
        Error_Handler();
    }
}

void rgb_led_driver_init(void)
{
    memset(&g_rgb_led_driver, 0, sizeof(g_rgb_led_driver));

    if (RGB_LED_TIMER.hdma[TIM_DMA_ID_UPDATE] != &hdma_tim3_up) {
        Error_Handler();
    }

    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_MODE_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_STATE_CHANNEL, 0U);

    if (HAL_TIM_PWM_Start(&RGB_LED_TIMER, RGB_LED_MODE_CHANNEL) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&RGB_LED_TIMER, RGB_LED_STATE_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
}

void rgb_led_driver_poll(void)
{
    if (!g_rgb_led_driver.refresh_pending || g_rgb_led_driver.frame_active) {
        return;
    }

    begin_dma_frame();
}

void rgb_led_driver_set_colors(rgb_led_color_t mode_led,
                               rgb_led_color_t state_led)
{
    if (colors_equal(g_rgb_led_driver.desired_mode_led, mode_led) &&
        colors_equal(g_rgb_led_driver.desired_state_led, state_led)) {
        return;
    }

    g_rgb_led_driver.desired_mode_led = mode_led;
    g_rgb_led_driver.desired_state_led = state_led;
    g_rgb_led_driver.refresh_pending = true;
}

void rgb_led_driver_on_tim_period_elapsed(TIM_HandleTypeDef *htim)
{
    if ((htim != &RGB_LED_TIMER) || !g_rgb_led_driver.frame_active) {
        return;
    }

    stop_dma_frame();

    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_MODE_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&RGB_LED_TIMER, RGB_LED_STATE_CHANNEL, 0U);

    g_rgb_led_driver.frame_active = false;
}
