#include "modules/display_view.h"

#include "modules/link_stats.h"
#include "modules/mode_control.h"

#include <stdio.h>
#include <string.h>

static void pad_line(char *line)
{
    size_t len = strlen(line);

    if (len >= DISPLAY_VIEW_LINE_LEN) {
        line[DISPLAY_VIEW_LINE_LEN] = '\0';
        return;
    }

    memset(&line[len], ' ', DISPLAY_VIEW_LINE_LEN - len);
    line[DISPLAY_VIEW_LINE_LEN] = '\0';
}

static uint32_t clamp_rate_999(uint32_t rate)
{
    return (rate > 999U) ? 999U : rate;
}

static uint32_t clamp_rate_99(uint32_t rate)
{
    return (rate > 99U) ? 99U : rate;
}

static void format_rf_rate_part(char *buffer, size_t size,
                                const char *label, uint32_t rate)
{
    rate = clamp_rate_999(rate);

    if (rate >= 100U) {
        (void)snprintf(buffer, size, "%s%03lu", label, (unsigned long)rate);
        return;
    }

    (void)snprintf(buffer, size, "%s%luHz", label, (unsigned long)rate);
}

static void build_status_lines(char *line0, char *line1)
{
    (void)snprintf(line0, DISPLAY_VIEW_LINE_LEN + 1U, "%s %s",
                   mode_control_get_module_name(),
                   mode_control_get_xbee_name());
    pad_line(line0);

    (void)snprintf(line1, DISPLAY_VIEW_LINE_LEN + 1U, "UP:%s DOWN:%s",
                   link_stats_get_status_code(LINK_STAT_UPLINK),
                   link_stats_get_status_code(LINK_STAT_DOWNLINK));
    pad_line(line1);
}

static void build_rate_lines(char *line0, char *line1)
{
    const link_stat_snapshot_t rf = link_stats_get_snapshot(LINK_STAT_RF);
    const link_stat_snapshot_t uplink = link_stats_get_snapshot(LINK_STAT_UPLINK);
    const link_stat_snapshot_t downlink = link_stats_get_snapshot(LINK_STAT_DOWNLINK);
    char tx_part[8];
    char rx_part[8];

    format_rf_rate_part(tx_part, sizeof(tx_part), "TX", rf.tx_hz);
    format_rf_rate_part(rx_part, sizeof(rx_part), "RX", rf.rx_hz);
    (void)snprintf(line0, DISPLAY_VIEW_LINE_LEN + 1U,
                   "RF:%s/%s", tx_part, rx_part);
    pad_line(line0);

    (void)snprintf(line1, DISPLAY_VIEW_LINE_LEN + 1U, "U:%lu/%lu D:%lu/%lu",
                   (unsigned long)clamp_rate_99(uplink.tx_hz),
                   (unsigned long)clamp_rate_99(uplink.rx_hz),
                   (unsigned long)clamp_rate_99(downlink.tx_hz),
                   (unsigned long)clamp_rate_99(downlink.rx_hz));
    pad_line(line1);
}

void display_view_build_startup(char *line0, char *line1)
{
    (void)snprintf(line0, DISPLAY_VIEW_LINE_LEN + 1U, "KONNICHIWA");
    line1[0] = '\0';
    pad_line(line0);
    pad_line(line1);
}

void display_view_build_error(char *line0, char *line1)
{
    (void)snprintf(line0, DISPLAY_VIEW_LINE_LEN + 1U, "ERROR OCCURED");
    line1[0] = '\0';
    pad_line(line0);
    pad_line(line1);
}

void display_view_build_mode(display_view_mode_t mode, char *line0, char *line1)
{
    if (mode == DISPLAY_VIEW_MODE_RATE) {
        build_rate_lines(line0, line1);
        return;
    }

    build_status_lines(line0, line1);
}

const char *display_view_mode_name(display_view_mode_t mode)
{
    return (mode == DISPLAY_VIEW_MODE_RATE) ? "rate" : "status";
}
