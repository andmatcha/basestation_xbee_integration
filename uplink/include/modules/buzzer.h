#ifndef UPLINK_BUZZER_H
#define UPLINK_BUZZER_H

#include <stdint.h>

typedef struct
{
    uint16_t frequency_hz;
    uint16_t duration_ms;
    uint16_t rest_ms;
} buzzer_note_t;

#define BUZZER_PITCH_REST               0U

#define BUZZER_PITCH_C3                 131U
#define BUZZER_PITCH_CS3                139U
#define BUZZER_PITCH_D3                 147U
#define BUZZER_PITCH_DS3                156U
#define BUZZER_PITCH_E3                 165U
#define BUZZER_PITCH_F3                 175U
#define BUZZER_PITCH_FS3                185U
#define BUZZER_PITCH_G3                 196U
#define BUZZER_PITCH_GS3                208U
#define BUZZER_PITCH_A3                 220U
#define BUZZER_PITCH_AS3                233U
#define BUZZER_PITCH_B3                 247U

#define BUZZER_PITCH_C4                 262U
#define BUZZER_PITCH_CS4                277U
#define BUZZER_PITCH_D4                 294U
#define BUZZER_PITCH_DS4                311U
#define BUZZER_PITCH_E4                 330U
#define BUZZER_PITCH_F4                 349U
#define BUZZER_PITCH_FS4                370U
#define BUZZER_PITCH_G4                 392U
#define BUZZER_PITCH_GS4                415U
#define BUZZER_PITCH_A4                 440U
#define BUZZER_PITCH_AS4                466U
#define BUZZER_PITCH_B4                 494U

#define BUZZER_PITCH_C5                 523U
#define BUZZER_PITCH_CS5                554U
#define BUZZER_PITCH_D5                 587U
#define BUZZER_PITCH_DS5                622U
#define BUZZER_PITCH_E5                 659U
#define BUZZER_PITCH_F5                 698U
#define BUZZER_PITCH_FS5                740U
#define BUZZER_PITCH_G5                 784U
#define BUZZER_PITCH_GS5                831U
#define BUZZER_PITCH_A5                 880U
#define BUZZER_PITCH_AS5                932U
#define BUZZER_PITCH_B5                 988U

#define BUZZER_PITCH_C6                 1047U
#define BUZZER_PITCH_CS6                1109U
#define BUZZER_PITCH_D6                 1175U
#define BUZZER_PITCH_DS6                1245U
#define BUZZER_PITCH_E6                 1319U
#define BUZZER_PITCH_F6                 1397U
#define BUZZER_PITCH_FS6                1480U
#define BUZZER_PITCH_G6                 1568U
#define BUZZER_PITCH_GS6                1661U
#define BUZZER_PITCH_A6                 1760U
#define BUZZER_PITCH_AS6                1865U
#define BUZZER_PITCH_B6                 1976U

#define BUZZER_PITCH_C7                 2093U
#define BUZZER_PITCH_CS7                2217U
#define BUZZER_PITCH_D7                 2349U
#define BUZZER_PITCH_DS7                2489U
#define BUZZER_PITCH_E7                 2637U
#define BUZZER_PITCH_F7                 2794U
#define BUZZER_PITCH_FS7                2960U
#define BUZZER_PITCH_G7                 3136U
#define BUZZER_PITCH_GS7                3322U
#define BUZZER_PITCH_A7                 3520U
#define BUZZER_PITCH_AS7                3729U
#define BUZZER_PITCH_B7                 3951U

#define BUZZER_WHOLE_NOTE_MS(bpm)       ((uint16_t)(240000UL / (uint32_t)(bpm)))
#define BUZZER_HALF_NOTE_MS(bpm)        ((uint16_t)(BUZZER_WHOLE_NOTE_MS(bpm) / 2U))
#define BUZZER_QUARTER_NOTE_MS(bpm)     ((uint16_t)(BUZZER_WHOLE_NOTE_MS(bpm) / 4U))
#define BUZZER_EIGHTH_NOTE_MS(bpm)      ((uint16_t)(BUZZER_WHOLE_NOTE_MS(bpm) / 8U))
#define BUZZER_SIXTEENTH_NOTE_MS(bpm)   ((uint16_t)(BUZZER_WHOLE_NOTE_MS(bpm) / 16U))
#define BUZZER_DOTTED_NOTE_MS(note_ms)  ((uint16_t)(((uint32_t)(note_ms) * 3U) / 2U))
#define BUZZER_TRIPLET_NOTE_MS(note_ms) ((uint16_t)(((uint32_t)(note_ms) * 2U) / 3U))

#define BUZZER_NOTE(pitch, duration_ms) \
    ((buzzer_note_t){(uint16_t)(pitch), (uint16_t)(duration_ms), 0U})
#define BUZZER_NOTE_WITH_REST(pitch, duration_ms, rest_ms) \
    ((buzzer_note_t){(uint16_t)(pitch), (uint16_t)(duration_ms), (uint16_t)(rest_ms)})
#define BUZZER_REST(duration_ms) \
    BUZZER_NOTE(BUZZER_PITCH_REST, (duration_ms))

void buzzer_init(void);
void buzzer_play_tone(uint32_t frequency_hz, uint32_t duration_ms);
void buzzer_play_melody(const buzzer_note_t *notes, uint32_t note_count);
void buzzer_play_startup_melody(void);
void buzzer_play_mode_switch_melody(void);

#endif /* UPLINK_BUZZER_H */
