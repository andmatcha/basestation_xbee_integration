#ifndef INTEGRATED_LINK_STATS_H
#define INTEGRATED_LINK_STATS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LINK_STAT_RF = 0,
    LINK_STAT_UPLINK,
    LINK_STAT_DOWNLINK,
    LINK_STAT_COUNT,
} link_stat_port_t;

typedef enum
{
    LINK_STAT_STATUS_NONE = 0,
    LINK_STAT_STATUS_OK,
    LINK_STAT_STATUS_FORMAT,
    LINK_STAT_STATUS_WRONG_PORT,
    LINK_STAT_STATUS_SYNC,
    LINK_STAT_STATUS_CRC,
    LINK_STAT_STATUS_OVERFLOW,
    LINK_STAT_STATUS_QUEUE_FULL,
    LINK_STAT_STATUS_ERROR,
} link_stat_status_t;

typedef struct
{
    uint32_t tx_hz;
    uint32_t rx_hz;
    bool tx_active;
    bool rx_active;
} link_stat_snapshot_t;

void link_stats_init(void);
void link_stats_poll(void);
void link_stats_note_tx(link_stat_port_t port);
void link_stats_note_rx(link_stat_port_t port);
void link_stats_note_status(link_stat_port_t port, link_stat_status_t status);
link_stat_snapshot_t link_stats_get_snapshot(link_stat_port_t port);
const char *link_stats_get_status_code(link_stat_port_t port);

#endif /* INTEGRATED_LINK_STATS_H */
