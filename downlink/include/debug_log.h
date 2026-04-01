#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <stdio.h>

#ifndef DEBUG_LOG_ENABLED
#define DEBUG_LOG_ENABLED 0
#endif

#if DEBUG_LOG_ENABLED
#define DEBUG_PRINTF(...) do { printf(__VA_ARGS__); } while (0)
#else
#define DEBUG_PRINTF(...) do { } while (0)
#endif

#endif /* DEBUG_LOG_H */
