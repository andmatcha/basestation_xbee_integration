#include "modules/link_stats.h"

#include "stm32f4xx_hal.h"

#include <string.h>

#define LINK_STATS_SAMPLE_MS        1000U
#define LINK_STATS_ACTIVITY_HOLD_MS 250U

typedef struct
{
    uint32_t total_tx;
    uint32_t total_rx;
    uint32_t sample_tx;
    uint32_t sample_rx;
    uint32_t tx_hz;
    uint32_t rx_hz;
    uint32_t last_tx_ms;
    uint32_t last_rx_ms;
} link_stat_counter_t;

typedef struct
{
    uint32_t sample_start_ms;
    link_stat_counter_t ports[LINK_STAT_COUNT];
} link_stats_context_t;

static link_stats_context_t g_stats;

static bool is_valid_port(link_stat_port_t port)
{
    return ((uint32_t)port) < LINK_STAT_COUNT;
}

void link_stats_init(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    g_stats.sample_start_ms = HAL_GetTick();
}

void link_stats_poll(void)
{
    const uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = now_ms - g_stats.sample_start_ms;

    if (elapsed_ms < LINK_STATS_SAMPLE_MS) {
        return;
    }

    if (elapsed_ms == 0U) {
        elapsed_ms = 1U;
    }

    for (uint32_t i = 0U; i < LINK_STAT_COUNT; i++) {
        g_stats.ports[i].tx_hz =
            (g_stats.ports[i].sample_tx * 1000U) / elapsed_ms;
        g_stats.ports[i].rx_hz =
            (g_stats.ports[i].sample_rx * 1000U) / elapsed_ms;
        g_stats.ports[i].sample_tx = 0U;
        g_stats.ports[i].sample_rx = 0U;
    }

    g_stats.sample_start_ms = now_ms;
}

void link_stats_note_tx(link_stat_port_t port)
{
    const uint32_t now_ms = HAL_GetTick();

    if (!is_valid_port(port)) {
        return;
    }

    g_stats.ports[port].total_tx++;
    g_stats.ports[port].sample_tx++;
    g_stats.ports[port].last_tx_ms = now_ms;
}

void link_stats_note_rx(link_stat_port_t port)
{
    const uint32_t now_ms = HAL_GetTick();

    if (!is_valid_port(port)) {
        return;
    }

    g_stats.ports[port].total_rx++;
    g_stats.ports[port].sample_rx++;
    g_stats.ports[port].last_rx_ms = now_ms;
}

link_stat_snapshot_t link_stats_get_snapshot(link_stat_port_t port)
{
    link_stat_snapshot_t snapshot = {0U, 0U, false, false};
    const uint32_t now_ms = HAL_GetTick();
    const link_stat_counter_t *counter;

    if (!is_valid_port(port)) {
        return snapshot;
    }

    counter = &g_stats.ports[port];
    snapshot.tx_hz = counter->tx_hz;
    snapshot.rx_hz = counter->rx_hz;
    snapshot.tx_active =
        (counter->last_tx_ms != 0U) &&
        ((now_ms - counter->last_tx_ms) <= LINK_STATS_ACTIVITY_HOLD_MS);
    snapshot.rx_active =
        (counter->last_rx_ms != 0U) &&
        ((now_ms - counter->last_rx_ms) <= LINK_STATS_ACTIVITY_HOLD_MS);

    return snapshot;
}
