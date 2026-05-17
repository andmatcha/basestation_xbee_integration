#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#ifndef DEBUG_LOG_ENABLED
#define DEBUG_LOG_ENABLED 0
#endif

#if DEBUG_LOG_ENABLED
void debug_log_printf(const char *format, ...);
void debug_log_set_downlink_raw_mode(bool enabled);
bool debug_log_is_downlink_raw_mode(void);
void debug_log_downlink_raw_bytes(const uint8_t *data, uint16_t len);

#define LOG(...) do { debug_log_printf(__VA_ARGS__); } while (0)
#else
static inline void debug_log_set_downlink_raw_mode(bool enabled)
{
    (void)enabled;
}

static inline bool debug_log_is_downlink_raw_mode(void)
{
    return false;
}

static inline void debug_log_downlink_raw_bytes(const uint8_t *data,
                                                uint16_t len)
{
    (void)data;
    (void)len;
}

#define LOG(...) do { } while (0)
#endif

#endif /* DEBUG_LOG_H */
