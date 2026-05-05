#ifndef INTEGRATED_DISPLAY_VIEW_H
#define INTEGRATED_DISPLAY_VIEW_H

#include <stdint.h>

#define DISPLAY_VIEW_LINE_LEN 16U

typedef enum
{
    DISPLAY_VIEW_MODE_STATUS = 0,
    DISPLAY_VIEW_MODE_RATE,
} display_view_mode_t;

void display_view_build_startup(char *line0, char *line1);
void display_view_build_error(char *line0, char *line1);
void display_view_build_mode(display_view_mode_t mode, char *line0, char *line1);
const char *display_view_mode_name(display_view_mode_t mode);

#endif /* INTEGRATED_DISPLAY_VIEW_H */
