#include "debug_log.h"

#if DEBUG_LOG_ENABLED

#include <stdarg.h>

#define RAW_LOG_BYTES_PER_LINE  16U

static volatile bool g_downlink_raw_mode;

void debug_log_printf(const char *format, ...)
{
    va_list args;

    if (g_downlink_raw_mode) {
        return;
    }

    va_start(args, format);
    (void)vprintf(format, args);
    va_end(args);
}

void debug_log_set_downlink_raw_mode(bool enabled)
{
    g_downlink_raw_mode = enabled;
}

bool debug_log_is_downlink_raw_mode(void)
{
    return g_downlink_raw_mode;
}

void debug_log_downlink_raw_bytes(const uint8_t *data, uint16_t len)
{
    uint16_t offset = 0U;

    if (!g_downlink_raw_mode || (data == NULL) || (len == 0U)) {
        return;
    }

    while (offset < len) {
        uint16_t line_len = (uint16_t)(len - offset);

        if (line_len > RAW_LOG_BYTES_PER_LINE) {
            line_len = RAW_LOG_BYTES_PER_LINE;
        }

        (void)printf("[integrated] downlink raw:");
        for (uint16_t i = 0U; i < line_len; i++) {
            (void)printf(" %02X", (unsigned int)data[offset + i]);
        }
        (void)printf("\r\n");

        offset = (uint16_t)(offset + line_len);
    }
}

#endif
