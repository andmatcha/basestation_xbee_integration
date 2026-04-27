#include "modules/buzzer.h"

#include "main.h"

#include <stdbool.h>

#define BUZZER_TIMER             htim13
#define BUZZER_TIMER_CHANNEL     TIM_CHANNEL_1
#define BUZZER_FREQUENCY_HZ      4000U
#define BUZZER_MIN_PERIOD_TICKS  2U
#define BUZZER_MAX_PERIOD_TICKS  65536UL
#define BUZZER_SHORT_MS          55U
#define BUZZER_GAP_MS            45U

typedef struct
{
    uint32_t counter_clock_hz;
    bool is_initialized;
} buzzer_context_t;

static buzzer_context_t g_buzzer;

extern TIM_HandleTypeDef htim13;

static uint32_t get_tim13_input_clock_hz(void)
{
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_HCLK_DIV1) {
        return pclk1_hz;
    }

    return pclk1_hz * 2U;
}

static void buzzer_set_silent(void)
{
    __HAL_TIM_SET_COMPARE(&BUZZER_TIMER, BUZZER_TIMER_CHANNEL, 0U);
    __HAL_TIM_SET_COUNTER(&BUZZER_TIMER, 0U);
}

static void buzzer_set_frequency(uint32_t frequency_hz)
{
    uint32_t period_ticks;
    uint32_t auto_reload;

    if (frequency_hz == 0U) {
        buzzer_set_silent();
        return;
    }

    if (g_buzzer.counter_clock_hz == 0U) {
        Error_Handler();
    }

    period_ticks = g_buzzer.counter_clock_hz / frequency_hz;
    if (period_ticks < BUZZER_MIN_PERIOD_TICKS) {
        period_ticks = BUZZER_MIN_PERIOD_TICKS;
    }
    if (period_ticks > BUZZER_MAX_PERIOD_TICKS) {
        period_ticks = BUZZER_MAX_PERIOD_TICKS;
    }

    auto_reload = period_ticks - 1U;
    __HAL_TIM_SET_AUTORELOAD(&BUZZER_TIMER, auto_reload);
    __HAL_TIM_SET_COMPARE(&BUZZER_TIMER, BUZZER_TIMER_CHANNEL, period_ticks / 2U);
    __HAL_TIM_SET_COUNTER(&BUZZER_TIMER, 0U);
}

static void buzzer_play_count(uint8_t count)
{
    if (!g_buzzer.is_initialized) {
        return;
    }

    for (uint8_t i = 0U; i < count; i++) {
        buzzer_set_frequency(BUZZER_FREQUENCY_HZ);
        HAL_Delay(BUZZER_SHORT_MS);
        buzzer_set_silent();

        if ((uint8_t)(i + 1U) < count) {
            HAL_Delay(BUZZER_GAP_MS);
        }
    }
}

void buzzer_init(void)
{
    g_buzzer.counter_clock_hz =
        get_tim13_input_clock_hz() / (BUZZER_TIMER.Init.Prescaler + 1U);
    g_buzzer.is_initialized = true;

    buzzer_set_silent();
    if (HAL_TIM_PWM_Start(&BUZZER_TIMER, BUZZER_TIMER_CHANNEL) != HAL_OK) {
        Error_Handler();
    }
}

void buzzer_play_short_beep(void)
{
    buzzer_play_count(1U);
}

void buzzer_play_double_beep(void)
{
    buzzer_play_count(2U);
}

void buzzer_play_startup_beeps(void)
{
    buzzer_play_count(3U);
}
