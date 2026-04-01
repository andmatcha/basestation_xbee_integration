#include "app.h"

#include "modules/input_source_selector.h"

void init(void)
{
    input_source_selector_init();
}

void poll(void)
{
    input_source_selector_poll();
}
