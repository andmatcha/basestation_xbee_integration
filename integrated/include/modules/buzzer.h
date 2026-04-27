#ifndef INTEGRATED_BUZZER_H
#define INTEGRATED_BUZZER_H

#include <stdint.h>

void buzzer_init(void);
void buzzer_play_short_beep(void);
void buzzer_play_double_beep(void);
void buzzer_play_startup_beeps(void);

#endif /* INTEGRATED_BUZZER_H */
