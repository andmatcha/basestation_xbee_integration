#include "modules/buzzer.h"

#include "main.h"

#include <stdbool.h>

#define BUZZER_TIMER              htim13
#define BUZZER_TIMER_CHANNEL      TIM_CHANNEL_1
#define BUZZER_MIN_PERIOD_TICKS   2U
#define BUZZER_MAX_PERIOD_TICKS   65536UL
#define BUZZER_STARTUP_BPM        180U
#define BUZZER_STARTUP_GAP_MS     20U
#define BUZZER_MODE_SWITCH_BPM    220U
#define BUZZER_MODE_SWITCH_GAP_MS 15U

typedef struct
{
    uint32_t counter_clock_hz;
    bool is_initialized;
} BuzzerContext;

static BuzzerContext g_buzzer;

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

void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms)
{
    if (!g_buzzer.is_initialized || (duration_ms == 0U)) {
        return;
    }

    buzzer_set_frequency(frequency_hz);
    HAL_Delay(duration_ms);
    buzzer_set_silent();
}

void buzzer_play_melody(const buzzer_note_t *notes, uint32_t note_count)
{
    if (!g_buzzer.is_initialized || (notes == NULL)) {
        return;
    }

    for (uint32_t i = 0U; i < note_count; i++) {
        buzzer_play_tone(notes[i].frequency_hz, notes[i].duration_ms);

        if (notes[i].rest_ms > 0U) {
            HAL_Delay(notes[i].rest_ms);
        }
    }
}

void buzzer_play_startup_melody(void)
{
    static const buzzer_note_t startup_melody[] = {
        BUZZER_NOTE_WITH_REST(BUZZER_PITCH_C5,
                              BUZZER_EIGHTH_NOTE_MS(BUZZER_STARTUP_BPM),
                              BUZZER_STARTUP_GAP_MS),
        BUZZER_NOTE_WITH_REST(BUZZER_PITCH_E5,
                              BUZZER_EIGHTH_NOTE_MS(BUZZER_STARTUP_BPM),
                              BUZZER_STARTUP_GAP_MS),
        BUZZER_NOTE(BUZZER_PITCH_G5,
                    BUZZER_DOTTED_NOTE_MS(
                        BUZZER_EIGHTH_NOTE_MS(BUZZER_STARTUP_BPM))),
    };

    buzzer_play_melody(startup_melody,
                       sizeof(startup_melody) / sizeof(startup_melody[0]));
}

void buzzer_play_mode_switch_melody(void)
{
    static const buzzer_note_t mode_switch_melody[] = {
        BUZZER_NOTE_WITH_REST(BUZZER_PITCH_C5,
                              BUZZER_SIXTEENTH_NOTE_MS(BUZZER_MODE_SWITCH_BPM),
                              BUZZER_MODE_SWITCH_GAP_MS),
        BUZZER_NOTE(BUZZER_PITCH_E5,
                    BUZZER_EIGHTH_NOTE_MS(BUZZER_MODE_SWITCH_BPM)),
    };

    buzzer_play_melody(mode_switch_melody,
                       sizeof(mode_switch_melody) / sizeof(mode_switch_melody[0]));
}
