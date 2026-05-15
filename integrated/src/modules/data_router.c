#include "modules/data_router.h"

#include "debug_log.h"
#include "main.h"
#include "modules/ac_packet_reducer.h"
#include "modules/display_manager.h"
#include "modules/link_stats.h"
#include "modules/mode_control.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define ROVER_IN_UART             huart1
#define MODULE_IN_UART            huart2
#define ROVER_OUT_UART            huart4
#define MODULE_OUT_UART           huart5
#define EXTERNAL_XBEE_UART        huart3
#define ONBOARD_XBEE_UART         huart6

#define RX_DMA_BUFFER_SIZE        256U
#define TEXT_LINE_BUFFER_SIZE     127U
#define TEXT_PACKET_MAX_LEN       128U
#define ROVER_LINE_MAX_LEN        TEXT_LINE_BUFFER_SIZE
#define ROVER_PACKET_MAX_LEN      TEXT_PACKET_MAX_LEN
#define SCIENCE_PACKET_MAX_LEN    TEXT_PACKET_MAX_LEN
#define ARM_PACKET_JF_SIZE        16U
#define UF_PACKET_V2_SIZE         40U
#define UF_PACKET_V2_CRC_OFFSET   38U
#define UF_PACKET_V2_PAYLOAD_LEN_OFFSET  5U
#define UF_PACKET_V2_PAYLOAD_MAX_LEN     32U
#define TX_QUEUE_DEPTH            24U
#define TX_FRAME_MAX_LEN          (TEXT_PACKET_MAX_LEN + 2U)

typedef struct __attribute__((packed))
{
    char header[2];
    uint8_t seq;
    uint8_t flags;
    uint16_t current[7];
    uint16_t angle[3];
    int16_t vel[3];
    uint8_t control_byte;
    int16_t base_rel_mm_j0;
    uint16_t auto_flags;
    uint16_t fault_code;
    uint16_t crc16;
} PacketAC_v6;

typedef char packet_ac_v6_size_must_match_reducer[
    (sizeof(PacketAC_v6) == AC_PACKET_REDUCER_AC_PACKET_LEN) ? 1 : -1];

typedef enum
{
    FRAME_TYPE_UPLINK_ROVER = 0,
    FRAME_TYPE_UPLINK_ARM_AC,
    FRAME_TYPE_UPLINK_ARM_JF,
    FRAME_TYPE_UPLINK_SCIENCE_TEXT,
    FRAME_TYPE_DOWNLINK_ROVER,
    FRAME_TYPE_DOWNLINK_ARM,
    FRAME_TYPE_DOWNLINK_UF,
    FRAME_TYPE_DOWNLINK_SCIENCE_TEXT,
} frame_type_t;

typedef enum
{
    ARM_SYNC_WAIT = 0,
    ARM_SYNC_WAIT_AC_C,
    ARM_SYNC_WAIT_JF_F,
    ARM_SYNC_COLLECT_AC,
    ARM_SYNC_COLLECT_JF,
} arm_rx_state_t;

typedef enum
{
    XBEE_FILTER_ROVER = 0,
    XBEE_FILTER_ARM,
    XBEE_FILTER_UF,
} xbee_filter_mode_t;

typedef enum
{
    TEXT_PACKET_ROUTE_NONE = 0,
    TEXT_PACKET_ROUTE_ROVER,
    TEXT_PACKET_ROUTE_SCIENCE,
} text_packet_route_t;

typedef struct
{
    uint8_t data[TX_FRAME_MAX_LEN];
    uint16_t len;
    uint8_t type;
} tx_frame_t;

typedef struct
{
    UART_HandleTypeDef *fixed_uart;
    UART_HandleTypeDef *tx_uart;
    tx_frame_t queue[TX_QUEUE_DEPTH];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t count;
    volatile bool busy;
    uint8_t dma_buffer[TX_FRAME_MAX_LEN];
    uint8_t current_type;
} tx_channel_t;

typedef struct
{
    module_mode_t module_mode;
    xbee_mode_t xbee_mode;
    uint32_t mode_generation;

    uint8_t rover_rx_dma[RX_DMA_BUFFER_SIZE];
    uint16_t rover_rx_pos;
    char rover_line[ROVER_LINE_MAX_LEN];
    uint16_t rover_line_len;
    bool rover_line_overflow;

    uint8_t module_rx_dma[RX_DMA_BUFFER_SIZE];
    uint16_t module_rx_pos;
    uint8_t arm_packet[sizeof(PacketAC_v6)];
    uint16_t arm_packet_len;
    arm_rx_state_t arm_rx_state;
    char science_line[TEXT_LINE_BUFFER_SIZE];
    uint16_t science_line_len;
    bool science_line_overflow;

    uint8_t xbee3_rx_dma[RX_DMA_BUFFER_SIZE];
    uint16_t xbee3_rx_pos;
    uint8_t xbee6_rx_dma[RX_DMA_BUFFER_SIZE];
    uint16_t xbee6_rx_pos;

    xbee_filter_mode_t xbee_filter_mode;
    bool xbee_rover_pending_j;
    bool xbee_rover_pending_j_sync_only;
    bool xbee_rover_pending_u;
    bool xbee_rover_pending_u_sync_only;
    uint8_t xbee_rover_buf[ROVER_PACKET_MAX_LEN];
    uint16_t xbee_rover_len;
    bool xbee_text_overflow;
    uint8_t xbee_arm_buf[ARM_PACKET_JF_SIZE];
    uint16_t xbee_arm_len;
    uint8_t xbee_uf_buf[UF_PACKET_V2_SIZE];
    uint16_t xbee_uf_len;

    tx_channel_t xbee_tx;
    tx_channel_t rover_out_tx;
    tx_channel_t module_out_tx;
} data_router_context_t;

static data_router_context_t g_router;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

static uint32_t enter_critical_section(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void exit_critical_section(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void report_error(void)
{
    display_manager_show_error();
}

static void note_uplink_status(link_stat_status_t status)
{
    link_stats_note_status(LINK_STAT_UPLINK, status);
}

static void note_rover_uplink_status(link_stat_status_t status)
{
    note_uplink_status(status);
    link_stats_note_status(LINK_STAT_ROVER_UPLINK, status);
}

static void note_module_uplink_status(link_stat_status_t status)
{
    note_uplink_status(status);
    link_stats_note_status(LINK_STAT_MODULE_UPLINK, status);
}

static void note_uplink_frame_status(frame_type_t type, link_stat_status_t status)
{
    switch (type) {
    case FRAME_TYPE_UPLINK_ROVER:
        note_rover_uplink_status(status);
        break;
    case FRAME_TYPE_UPLINK_ARM_AC:
    case FRAME_TYPE_UPLINK_ARM_JF:
    case FRAME_TYPE_UPLINK_SCIENCE_TEXT:
        note_module_uplink_status(status);
        break;
    default:
        note_uplink_status(status);
        break;
    }
}

static void note_uplink_frame_rx(frame_type_t type)
{
    switch (type) {
    case FRAME_TYPE_UPLINK_ROVER:
        link_stats_note_rx(LINK_STAT_ROVER_UPLINK);
        break;
    case FRAME_TYPE_UPLINK_ARM_AC:
    case FRAME_TYPE_UPLINK_ARM_JF:
    case FRAME_TYPE_UPLINK_SCIENCE_TEXT:
        link_stats_note_rx(LINK_STAT_MODULE_UPLINK);
        break;
    default:
        break;
    }
}

static void note_uplink_frame_tx(frame_type_t type)
{
    switch (type) {
    case FRAME_TYPE_UPLINK_ROVER:
        link_stats_note_tx(LINK_STAT_ROVER_UPLINK);
        break;
    case FRAME_TYPE_UPLINK_ARM_AC:
    case FRAME_TYPE_UPLINK_ARM_JF:
    case FRAME_TYPE_UPLINK_SCIENCE_TEXT:
        link_stats_note_tx(LINK_STAT_MODULE_UPLINK);
        break;
    default:
        break;
    }
}

static void note_downlink_status(link_stat_status_t status)
{
    link_stats_note_status(LINK_STAT_DOWNLINK, status);
}

static void note_rover_downlink_status(link_stat_status_t status)
{
    note_downlink_status(status);
    link_stats_note_status(LINK_STAT_ROVER_DOWNLINK, status);
}

static void note_module_downlink_status(link_stat_status_t status)
{
    note_downlink_status(status);
    link_stats_note_status(LINK_STAT_MODULE_DOWNLINK, status);
}

static void note_downlink_frame_status(frame_type_t type, link_stat_status_t status)
{
    switch (type) {
    case FRAME_TYPE_DOWNLINK_ROVER:
        note_rover_downlink_status(status);
        break;
    case FRAME_TYPE_DOWNLINK_ARM:
    case FRAME_TYPE_DOWNLINK_UF:
    case FRAME_TYPE_DOWNLINK_SCIENCE_TEXT:
        note_module_downlink_status(status);
        break;
    default:
        note_downlink_status(status);
        break;
    }
}

static void note_downlink_frame_tx(frame_type_t type)
{
    switch (type) {
    case FRAME_TYPE_DOWNLINK_ROVER:
        link_stats_note_tx(LINK_STAT_ROVER_DOWNLINK);
        break;
    case FRAME_TYPE_DOWNLINK_ARM:
    case FRAME_TYPE_DOWNLINK_UF:
    case FRAME_TYPE_DOWNLINK_SCIENCE_TEXT:
        link_stats_note_tx(LINK_STAT_MODULE_DOWNLINK);
        break;
    default:
        break;
    }
}

static void note_tx_error_status(frame_type_t type)
{
    switch (type) {
    case FRAME_TYPE_UPLINK_ROVER:
    case FRAME_TYPE_UPLINK_ARM_AC:
    case FRAME_TYPE_UPLINK_ARM_JF:
    case FRAME_TYPE_UPLINK_SCIENCE_TEXT:
        note_uplink_frame_status(type, LINK_STAT_STATUS_ERROR);
        break;
    case FRAME_TYPE_DOWNLINK_ROVER:
    case FRAME_TYPE_DOWNLINK_ARM:
    case FRAME_TYPE_DOWNLINK_UF:
    case FRAME_TYPE_DOWNLINK_SCIENCE_TEXT:
        note_downlink_frame_status(type, LINK_STAT_STATUS_ERROR);
        break;
    default:
        break;
    }
}

#if DEBUG_LOG_ENABLED
static const char *get_uart_name(const UART_HandleTypeDef *huart)
{
    if (huart == &ROVER_IN_UART) {
        return "USB1 rover uplink";
    }
    if (huart == &MODULE_IN_UART) {
        return "USB2 module uplink";
    }
    if (huart == &ROVER_OUT_UART) {
        return "USB3 rover/arm downlink";
    }
    if (huart == &MODULE_OUT_UART) {
        return "USB4 module/arm downlink";
    }
    if (huart == &EXTERNAL_XBEE_UART) {
        return "USART3 external xbee";
    }
    if (huart == &ONBOARD_XBEE_UART) {
        return "USART6 onboard xbee";
    }

    return "unknown uart";
}
#endif

static HAL_StatusTypeDef start_circular_reception(UART_HandleTypeDef *huart,
                                                  uint8_t *buffer,
                                                  uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, buffer, size);

    if ((status == HAL_OK) && (huart->hdmarx != NULL)) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);
    }

    return status;
}

static void restart_circular_reception(UART_HandleTypeDef *huart,
                                       uint8_t *buffer,
                                       uint16_t size,
                                       uint16_t *position)
{
    (void)HAL_UART_DMAStop(huart);

    if (start_circular_reception(huart, buffer, size) != HAL_OK) {
        report_error();
        Error_Handler();
    }

    if (position != NULL) {
        *position = 0U;
    }
}

static uint16_t get_dma_position(const UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->hdmarx == NULL) {
        return 0U;
    }

    return (uint16_t)(size - __HAL_DMA_GET_COUNTER(huart->hdmarx));
}

static void tx_channel_init(tx_channel_t *channel, UART_HandleTypeDef *fixed_uart)
{
    memset(channel, 0, sizeof(*channel));
    channel->fixed_uart = fixed_uart;
}

static bool tx_channel_enqueue(tx_channel_t *channel,
                               frame_type_t type,
                               const uint8_t *data,
                               uint16_t len)
{
    tx_frame_t *frame;
    uint32_t primask;

    if ((channel == NULL) || (data == NULL) ||
        (len == 0U) || (len > TX_FRAME_MAX_LEN)) {
        return false;
    }

    primask = enter_critical_section();
    if (channel->count >= TX_QUEUE_DEPTH) {
        exit_critical_section(primask);
        return false;
    }

    frame = &channel->queue[channel->tail];
    memcpy(frame->data, data, len);
    frame->len = len;
    frame->type = (uint8_t)type;

    channel->tail = (uint16_t)((channel->tail + 1U) % TX_QUEUE_DEPTH);
    channel->count++;
    exit_critical_section(primask);
    return true;
}

static void note_tx_start(frame_type_t type, const UART_HandleTypeDef *huart,
                          uint16_t len)
{
    switch (type) {
    case FRAME_TYPE_UPLINK_ROVER:
        link_stats_note_tx(LINK_STAT_RF);
        link_stats_note_tx(LINK_STAT_UPLINK);
        note_uplink_frame_tx(type);
        note_uplink_frame_status(type, LINK_STAT_STATUS_OK);
        LOG("[integrated] uplink rover tx -> %s len=%u\r\n",
            get_uart_name(huart), (unsigned int)len);
        break;
    case FRAME_TYPE_UPLINK_ARM_AC:
    case FRAME_TYPE_UPLINK_ARM_JF:
    case FRAME_TYPE_UPLINK_SCIENCE_TEXT:
        link_stats_note_tx(LINK_STAT_RF);
        link_stats_note_tx(LINK_STAT_UPLINK);
        note_uplink_frame_tx(type);
        note_uplink_frame_status(type, LINK_STAT_STATUS_OK);
        LOG("[integrated] uplink %s tx -> %s len=%u\r\n",
            mode_control_get_module_name(),
            get_uart_name(huart), (unsigned int)len);
        break;
    case FRAME_TYPE_DOWNLINK_ROVER:
        link_stats_note_tx(LINK_STAT_DOWNLINK);
        note_downlink_frame_tx(type);
        note_downlink_frame_status(type, LINK_STAT_STATUS_OK);
        LOG("[integrated] downlink rover tx -> %s len=%u\r\n",
            get_uart_name(huart), (unsigned int)len);
        break;
    case FRAME_TYPE_DOWNLINK_ARM:
        link_stats_note_tx(LINK_STAT_DOWNLINK);
        note_downlink_frame_tx(type);
        note_downlink_frame_status(type, LINK_STAT_STATUS_OK);
        LOG("[integrated] downlink %s tx -> %s len=%u\r\n",
            mode_control_get_module_name(),
            get_uart_name(huart), (unsigned int)len);
        break;
    case FRAME_TYPE_DOWNLINK_UF:
        link_stats_note_tx(LINK_STAT_DOWNLINK);
        note_downlink_frame_tx(type);
        note_downlink_frame_status(type, LINK_STAT_STATUS_OK);
        LOG("[integrated] downlink UF v2 tx -> %s len=%u\r\n",
            get_uart_name(huart), (unsigned int)len);
        break;
    case FRAME_TYPE_DOWNLINK_SCIENCE_TEXT:
        link_stats_note_tx(LINK_STAT_DOWNLINK);
        note_downlink_frame_tx(type);
        note_downlink_frame_status(type, LINK_STAT_STATUS_OK);
        LOG("[integrated] downlink science text tx -> %s len=%u\r\n",
            get_uart_name(huart), (unsigned int)len);
        break;
    default:
        break;
    }
}

static void tx_channel_pump(tx_channel_t *channel, UART_HandleTypeDef *selected_uart)
{
    UART_HandleTypeDef *uart;
    uint16_t frame_len;
    frame_type_t frame_type;
    ac_packet_reducer_result_t reduce_result = AC_PACKET_REDUCER_RESULT_OK;
    bool dropped_frame = false;
    uint32_t primask;

    if (channel == NULL) {
        return;
    }

    uart = (selected_uart != NULL) ? selected_uart : channel->fixed_uart;
    if (uart == NULL) {
        return;
    }

    primask = enter_critical_section();
    if (channel->busy || (channel->count == 0U)) {
        exit_critical_section(primask);
        return;
    }

    frame_len = channel->queue[channel->head].len;
    frame_type = (frame_type_t)channel->queue[channel->head].type;

    if (frame_type == FRAME_TYPE_UPLINK_ARM_AC) {
        size_t reduced_len = 0U;

        reduce_result = ac_packet_reducer_reduce_for_xbee(
            channel->queue[channel->head].data,
            frame_len,
            channel->dma_buffer,
            sizeof(channel->dma_buffer),
            &reduced_len);

        if (reduce_result == AC_PACKET_REDUCER_RESULT_OK) {
            frame_len = (uint16_t)reduced_len;
        } else {
            if (channel->count > 0U) {
                channel->head = (uint16_t)((channel->head + 1U) % TX_QUEUE_DEPTH);
                channel->count--;
            }
            dropped_frame = true;
        }
    } else {
        memcpy(channel->dma_buffer, channel->queue[channel->head].data, frame_len);
    }

    if (dropped_frame) {
        exit_critical_section(primask);
        note_uplink_frame_status(frame_type, LINK_STAT_STATUS_ERROR);
        LOG("[integrated] dropped arm AC packet before XBee tx: %s\r\n",
            ac_packet_reducer_result_name(reduce_result));
        return;
    }

    channel->busy = true;
    channel->tx_uart = uart;
    channel->current_type = (uint8_t)frame_type;
    exit_critical_section(primask);

    if (HAL_UART_Transmit_DMA(uart, channel->dma_buffer, frame_len) != HAL_OK) {
        primask = enter_critical_section();
        channel->busy = false;
        channel->tx_uart = NULL;
        exit_critical_section(primask);
        LOG("[integrated] uart tx start failed on %s\r\n", get_uart_name(uart));
        note_tx_error_status(frame_type);
        report_error();
        return;
    }

    note_tx_start(frame_type, uart, frame_len);
}

static bool tx_channel_on_complete(tx_channel_t *channel, UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((channel == NULL) || (huart != channel->tx_uart)) {
        return false;
    }

    primask = enter_critical_section();
    if (channel->count > 0U) {
        channel->head = (uint16_t)((channel->head + 1U) % TX_QUEUE_DEPTH);
        channel->count--;
    }
    channel->busy = false;
    channel->tx_uart = NULL;
    exit_critical_section(primask);
    return true;
}

static bool tx_channel_on_error(tx_channel_t *channel, UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if ((channel == NULL) || (huart != channel->tx_uart)) {
        return false;
    }

    (void)HAL_UART_DMAStop(huart);

    primask = enter_critical_section();
    channel->busy = false;
    channel->tx_uart = NULL;
    exit_critical_section(primask);
    return true;
}

static bool is_hex_digit(uint8_t byte)
{
    return ((byte >= '0') && (byte <= '9')) ||
           ((byte >= 'A') && (byte <= 'F')) ||
           ((byte >= 'a') && (byte <= 'f'));
}

static unsigned long hex_digit_to_value(uint8_t byte)
{
    if ((byte >= '0') && (byte <= '9')) {
        return (unsigned long)(byte - '0');
    }

    if ((byte >= 'A') && (byte <= 'F')) {
        return (unsigned long)(byte - 'A' + 10U);
    }

    return (unsigned long)(byte - 'a' + 10U);
}

static bool validate_rover_text(const uint8_t *packet, uint16_t length)
{
    uint16_t idx = 2U;
    unsigned long can_id = 0UL;

    if (length < 5U) {
        return false;
    }

    if ((packet[0] != '0') || ((packet[1] != 'x') && (packet[1] != 'X'))) {
        return false;
    }

    if ((packet[idx] != '3') && (packet[idx] != '4')) {
        return false;
    }

    while ((idx < length) && (packet[idx] != ',')) {
        if (!is_hex_digit(packet[idx])) {
            return false;
        }

        can_id = (can_id * 16UL) + hex_digit_to_value(packet[idx]);
        if (can_id > 0x7FFUL) {
            return false;
        }

        idx++;
    }

    if ((idx >= length) || (packet[idx] != ',')) {
        return false;
    }

    idx++;
    return idx < length;
}

static bool validate_science_mode_text_packet(const uint8_t *packet,
                                              uint16_t length,
                                              uint8_t first_id_digit)
{
    if (length < 7U) {
        return false;
    }

    if ((packet[0] != '0') || ((packet[1] != 'x') && (packet[1] != 'X'))) {
        return false;
    }

    if (packet[2] != first_id_digit) {
        return false;
    }

    if (!is_hex_digit(packet[3]) || !is_hex_digit(packet[4]) ||
        (packet[5] != ',')) {
        return false;
    }

    return true;
}

static bool validate_science_mode_rover_text(const uint8_t *packet, uint16_t length)
{
    return validate_science_mode_text_packet(packet, length, '3') ||
           validate_science_mode_text_packet(packet, length, '4');
}

static bool validate_science_text(const uint8_t *packet, uint16_t length)
{
    return validate_science_mode_text_packet(packet, length, '5');
}

static text_packet_route_t classify_science_mode_text_packet(const uint8_t *packet,
                                                             uint16_t length)
{
    if (validate_science_text(packet, length)) {
        return TEXT_PACKET_ROUTE_SCIENCE;
    }

    if (validate_science_mode_rover_text(packet, length)) {
        return TEXT_PACKET_ROUTE_ROVER;
    }

    return TEXT_PACKET_ROUTE_NONE;
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static bool validate_ac_packet(const uint8_t *raw_packet)
{
    PacketAC_v6 packet;
    uint16_t crc_calc;

    memcpy(&packet, raw_packet, sizeof(packet));

    if ((packet.header[0] != 'A') || (packet.header[1] != 'C')) {
        return false;
    }

    crc_calc = crc16_ccitt_false(raw_packet, offsetof(PacketAC_v6, crc16));
    return crc_calc == packet.crc16;
}

static void enqueue_uplink_frame(frame_type_t type, const uint8_t *data,
                                 uint16_t len)
{
    if (!tx_channel_enqueue(&g_router.xbee_tx, type, data, len)) {
        LOG("[integrated] uplink tx queue full\r\n");
        note_uplink_frame_status(type, LINK_STAT_STATUS_QUEUE_FULL);
        report_error();
        return;
    }

    link_stats_note_rx(LINK_STAT_UPLINK);
    note_uplink_frame_rx(type);
    note_uplink_frame_status(type, LINK_STAT_STATUS_OK);
}

static void enqueue_text_uplink_frame(frame_type_t type,
                                      const uint8_t *line,
                                      uint16_t line_len)
{
    uint8_t tx_frame[TX_FRAME_MAX_LEN];

    if ((line_len + 2U) > sizeof(tx_frame)) {
        LOG("[integrated] text uplink line too long len=%u\r\n",
            (unsigned int)line_len);
        note_uplink_frame_status(type, LINK_STAT_STATUS_OVERFLOW);
        return;
    }

    memcpy(tx_frame, line, line_len);
    tx_frame[line_len] = '\r';
    tx_frame[line_len + 1U] = '\n';
    enqueue_uplink_frame(type, tx_frame, (uint16_t)(line_len + 2U));
}

static void consume_rover_uplink_byte(uint8_t byte)
{
    bool valid_line;

    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (g_router.rover_line_overflow) {
            g_router.rover_line_len = 0U;
            g_router.rover_line_overflow = false;
            return;
        }

        if (g_router.rover_line_len == 0U) {
            return;
        }

        valid_line =
            (mode_control_get_module_mode() == MODULE_MODE_SCIENCE)
                ? validate_science_mode_rover_text((const uint8_t *)g_router.rover_line,
                                                   g_router.rover_line_len)
                : validate_rover_text((const uint8_t *)g_router.rover_line,
                                      g_router.rover_line_len);
        if (valid_line) {
            enqueue_text_uplink_frame(FRAME_TYPE_UPLINK_ROVER,
                                      (const uint8_t *)g_router.rover_line,
                                      g_router.rover_line_len);
        } else if ((mode_control_get_module_mode() == MODULE_MODE_SCIENCE) &&
                   validate_science_text((const uint8_t *)g_router.rover_line,
                                         g_router.rover_line_len)) {
            LOG("[integrated] science uplink packet on rover port len=%u\r\n",
                (unsigned int)g_router.rover_line_len);
            note_rover_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else if ((g_router.rover_line[0] == 'A') ||
                   (g_router.rover_line[0] == 'J')) {
            LOG("[integrated] module uplink packet on rover port len=%u\r\n",
                (unsigned int)g_router.rover_line_len);
            note_rover_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else {
            LOG("[integrated] rover uplink rejected len=%u\r\n",
                (unsigned int)g_router.rover_line_len);
            note_rover_uplink_status(LINK_STAT_STATUS_FORMAT);
        }

        g_router.rover_line_len = 0U;
        return;
    }

    if (g_router.rover_line_overflow) {
        return;
    }

    if (g_router.rover_line_len >= (ROVER_LINE_MAX_LEN - 1U)) {
        g_router.rover_line_len = 0U;
        g_router.rover_line_overflow = true;
        LOG("[integrated] rover uplink line overflow\r\n");
        note_rover_uplink_status(LINK_STAT_STATUS_OVERFLOW);
        return;
    }

    if (g_router.rover_line_len == 0U) {
        if ((byte == 'A') || (byte == 'J')) {
            note_rover_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else if (byte != '0') {
            note_rover_uplink_status(LINK_STAT_STATUS_FORMAT);
        }
    }

    g_router.rover_line[g_router.rover_line_len++] = (char)byte;
}

static void reset_module_uplink_parser(void)
{
    g_router.arm_packet_len = 0U;
    g_router.arm_rx_state = ARM_SYNC_WAIT;
    g_router.science_line_len = 0U;
    g_router.science_line_overflow = false;
}

static void enqueue_arm_uplink_packet(frame_type_t type, const uint8_t *packet,
                                      uint16_t len)
{
    enqueue_uplink_frame(type, packet, len);
}

static void consume_science_uplink_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (g_router.science_line_overflow) {
            g_router.science_line_len = 0U;
            g_router.science_line_overflow = false;
            return;
        }

        if (g_router.science_line_len == 0U) {
            return;
        }

        if (validate_science_text((const uint8_t *)g_router.science_line,
                                  g_router.science_line_len)) {
            enqueue_text_uplink_frame(FRAME_TYPE_UPLINK_SCIENCE_TEXT,
                                      (const uint8_t *)g_router.science_line,
                                      g_router.science_line_len);
        } else if (validate_science_mode_rover_text(
                       (const uint8_t *)g_router.science_line,
                       g_router.science_line_len)) {
            LOG("[integrated] rover uplink packet on module port len=%u\r\n",
                (unsigned int)g_router.science_line_len);
            note_module_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else if ((g_router.science_line[0] == 'A') ||
                   (g_router.science_line[0] == 'J')) {
            LOG("[integrated] arm uplink packet on science port len=%u\r\n",
                (unsigned int)g_router.science_line_len);
            note_module_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else {
            LOG("[integrated] science uplink rejected len=%u\r\n",
                (unsigned int)g_router.science_line_len);
            note_module_uplink_status(LINK_STAT_STATUS_FORMAT);
        }

        g_router.science_line_len = 0U;
        return;
    }

    if (g_router.science_line_overflow) {
        return;
    }

    if (g_router.science_line_len >= (TEXT_LINE_BUFFER_SIZE - 1U)) {
        g_router.science_line_len = 0U;
        g_router.science_line_overflow = true;
        LOG("[integrated] science uplink line overflow\r\n");
        note_module_uplink_status(LINK_STAT_STATUS_OVERFLOW);
        return;
    }

    if (g_router.science_line_len == 0U) {
        if ((byte == 'A') || (byte == 'J')) {
            note_module_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else if (byte != '0') {
            note_module_uplink_status(LINK_STAT_STATUS_FORMAT);
        }
    }

    g_router.science_line[g_router.science_line_len++] = byte;
}

static void consume_module_uplink_byte(uint8_t byte)
{
    if (mode_control_get_module_mode() == MODULE_MODE_SCIENCE) {
        consume_science_uplink_byte(byte);
        return;
    }

    switch (g_router.arm_rx_state) {
    case ARM_SYNC_WAIT:
        if (byte == 'A') {
            g_router.arm_packet[0] = byte;
            g_router.arm_packet_len = 1U;
            g_router.arm_rx_state = ARM_SYNC_WAIT_AC_C;
        } else if (byte == 'J') {
            g_router.arm_packet[0] = byte;
            g_router.arm_packet_len = 1U;
            g_router.arm_rx_state = ARM_SYNC_WAIT_JF_F;
        } else if ((byte == '0') || (byte == 'x') || (byte == 'X') ||
                   (byte == ',') || (byte == '-') || (byte == '+') ||
                   is_hex_digit(byte)) {
            note_module_uplink_status(LINK_STAT_STATUS_WRONG_PORT);
        } else {
            note_module_uplink_status(LINK_STAT_STATUS_FORMAT);
        }
        break;

    case ARM_SYNC_WAIT_AC_C:
        if (byte == 'C') {
            g_router.arm_packet[g_router.arm_packet_len++] = byte;
            g_router.arm_rx_state = ARM_SYNC_COLLECT_AC;
        } else if (byte == 'A') {
            g_router.arm_packet[0] = byte;
            g_router.arm_packet_len = 1U;
        } else if (byte == 'J') {
            g_router.arm_packet[0] = byte;
            g_router.arm_packet_len = 1U;
            g_router.arm_rx_state = ARM_SYNC_WAIT_JF_F;
        } else {
            note_module_uplink_status(LINK_STAT_STATUS_SYNC);
            reset_module_uplink_parser();
        }
        break;

    case ARM_SYNC_WAIT_JF_F:
        if (byte == 'F') {
            g_router.arm_packet[g_router.arm_packet_len++] = byte;
            g_router.arm_rx_state = ARM_SYNC_COLLECT_JF;
        } else if (byte == 'J') {
            g_router.arm_packet[0] = byte;
            g_router.arm_packet_len = 1U;
        } else if (byte == 'A') {
            g_router.arm_packet[0] = byte;
            g_router.arm_packet_len = 1U;
            g_router.arm_rx_state = ARM_SYNC_WAIT_AC_C;
        } else {
            note_module_uplink_status(LINK_STAT_STATUS_SYNC);
            reset_module_uplink_parser();
        }
        break;

    case ARM_SYNC_COLLECT_AC:
        g_router.arm_packet[g_router.arm_packet_len++] = byte;
        if (g_router.arm_packet_len >= sizeof(PacketAC_v6)) {
            if (validate_ac_packet(g_router.arm_packet)) {
                enqueue_arm_uplink_packet(FRAME_TYPE_UPLINK_ARM_AC,
                                          g_router.arm_packet,
                                          sizeof(PacketAC_v6));
            } else {
                LOG("[integrated] %s AC packet rejected by crc\r\n",
                    mode_control_get_module_name());
                note_module_uplink_status(LINK_STAT_STATUS_CRC);
            }
            reset_module_uplink_parser();
        }
        break;

    case ARM_SYNC_COLLECT_JF:
        g_router.arm_packet[g_router.arm_packet_len++] = byte;
        if (g_router.arm_packet_len >= ARM_PACKET_JF_SIZE) {
            enqueue_arm_uplink_packet(FRAME_TYPE_UPLINK_ARM_JF,
                                      g_router.arm_packet,
                                      ARM_PACKET_JF_SIZE);
            reset_module_uplink_parser();
        }
        break;

    default:
        reset_module_uplink_parser();
        break;
    }
}

static void poll_rover_uplink_dma(void)
{
    const uint16_t dma_pos =
        get_dma_position(&ROVER_IN_UART, sizeof(g_router.rover_rx_dma));

    while (g_router.rover_rx_pos != dma_pos) {
        consume_rover_uplink_byte(g_router.rover_rx_dma[g_router.rover_rx_pos]);
        g_router.rover_rx_pos++;
        if (g_router.rover_rx_pos >= sizeof(g_router.rover_rx_dma)) {
            g_router.rover_rx_pos = 0U;
        }
    }
}

static void poll_module_uplink_dma(void)
{
    const uint16_t dma_pos =
        get_dma_position(&MODULE_IN_UART, sizeof(g_router.module_rx_dma));

    while (g_router.module_rx_pos != dma_pos) {
        consume_module_uplink_byte(g_router.module_rx_dma[g_router.module_rx_pos]);
        g_router.module_rx_pos++;
        if (g_router.module_rx_pos >= sizeof(g_router.module_rx_dma)) {
            g_router.module_rx_pos = 0U;
        }
    }
}

static void reset_xbee_downlink_parser(void)
{
    g_router.xbee_filter_mode = XBEE_FILTER_ROVER;
    g_router.xbee_rover_pending_j = false;
    g_router.xbee_rover_pending_j_sync_only = false;
    g_router.xbee_rover_pending_u = false;
    g_router.xbee_rover_pending_u_sync_only = false;
    g_router.xbee_rover_len = 0U;
    g_router.xbee_text_overflow = false;
    g_router.xbee_arm_len = 0U;
    g_router.xbee_uf_len = 0U;
}

static void note_xbee_packet_rx(link_stat_port_t downlink_port)
{
    link_stats_note_rx(LINK_STAT_RF);
    link_stats_note_rx(LINK_STAT_DOWNLINK);
    link_stats_note_rx(downlink_port);
    note_downlink_status(LINK_STAT_STATUS_OK);
    link_stats_note_status(downlink_port, LINK_STAT_STATUS_OK);
}

static void note_science_mode_downlink_status(const uint8_t *packet,
                                              uint16_t len,
                                              link_stat_status_t status)
{
    if ((len > 2U) &&
        (packet[0] == '0') &&
        ((packet[1] == 'x') || (packet[1] == 'X'))) {
        if (packet[2] == '5') {
            note_module_downlink_status(status);
            return;
        }

        if ((packet[2] == '3') || (packet[2] == '4')) {
            note_rover_downlink_status(status);
            return;
        }
    }

    note_downlink_status(status);
}

static void route_downlink_rover_packet(const uint8_t *packet, uint16_t len)
{
    uint8_t tx_frame[TX_FRAME_MAX_LEN];
    tx_channel_t *tx_channel =
        (mode_control_get_module_mode() == MODULE_MODE_ARM)
            ? &g_router.module_out_tx
            : &g_router.rover_out_tx;

    if (!validate_rover_text(packet, len)) {
        LOG("[integrated] rover downlink rejected len=%u\r\n", (unsigned int)len);
        note_rover_downlink_status(LINK_STAT_STATUS_FORMAT);
        return;
    }

    memcpy(tx_frame, packet, len);
    tx_frame[len] = '\r';
    tx_frame[len + 1U] = '\n';

    if (!tx_channel_enqueue(tx_channel, FRAME_TYPE_DOWNLINK_ROVER,
                            tx_frame, (uint16_t)(len + 2U))) {
        LOG("[integrated] rover downlink tx queue full\r\n");
        note_rover_downlink_status(LINK_STAT_STATUS_QUEUE_FULL);
        report_error();
        return;
    }

    note_xbee_packet_rx(LINK_STAT_ROVER_DOWNLINK);
}

static void route_downlink_science_mode_text_packet(const uint8_t *packet, uint16_t len)
{
    uint8_t tx_frame[TX_FRAME_MAX_LEN];
    tx_channel_t *tx_channel;
    frame_type_t frame_type;
    text_packet_route_t route;

    route = classify_science_mode_text_packet(packet, len);
    if (route == TEXT_PACKET_ROUTE_NONE) {
        LOG("[integrated] science-mode downlink rejected len=%u\r\n",
            (unsigned int)len);
        note_science_mode_downlink_status(packet, len, LINK_STAT_STATUS_FORMAT);
        return;
    }

    if ((len + 2U) > sizeof(tx_frame)) {
        LOG("[integrated] science-mode downlink too long len=%u\r\n",
            (unsigned int)len);
        note_science_mode_downlink_status(packet, len, LINK_STAT_STATUS_OVERFLOW);
        return;
    }

    memcpy(tx_frame, packet, len);
    tx_frame[len] = '\r';
    tx_frame[len + 1U] = '\n';

    if (route == TEXT_PACKET_ROUTE_SCIENCE) {
        tx_channel = &g_router.module_out_tx;
        frame_type = FRAME_TYPE_DOWNLINK_SCIENCE_TEXT;
    } else {
        tx_channel = &g_router.rover_out_tx;
        frame_type = FRAME_TYPE_DOWNLINK_ROVER;
    }

    if (!tx_channel_enqueue(tx_channel, frame_type, tx_frame,
                            (uint16_t)(len + 2U))) {
        LOG("[integrated] science-mode downlink tx queue full\r\n");
        note_downlink_frame_status(frame_type, LINK_STAT_STATUS_QUEUE_FULL);
        report_error();
        return;
    }

    note_xbee_packet_rx((route == TEXT_PACKET_ROUTE_SCIENCE)
                            ? LINK_STAT_MODULE_DOWNLINK
                            : LINK_STAT_ROVER_DOWNLINK);
}

static void route_downlink_arm_packet(const uint8_t *packet)
{
    if (!tx_channel_enqueue(&g_router.module_out_tx, FRAME_TYPE_DOWNLINK_ARM,
                            packet, ARM_PACKET_JF_SIZE)) {
        LOG("[integrated] arm downlink tx queue full\r\n");
        note_module_downlink_status(LINK_STAT_STATUS_QUEUE_FULL);
        report_error();
        return;
    }

    note_xbee_packet_rx(LINK_STAT_MODULE_DOWNLINK);
}

static void route_downlink_uf_packet(const uint8_t *packet)
{
    if (!tx_channel_enqueue(&g_router.module_out_tx, FRAME_TYPE_DOWNLINK_UF,
                            packet, UF_PACKET_V2_SIZE)) {
        LOG("[integrated] UF v2 downlink tx queue full\r\n");
        note_module_downlink_status(LINK_STAT_STATUS_QUEUE_FULL);
        report_error();
        return;
    }

    note_xbee_packet_rx(LINK_STAT_MODULE_DOWNLINK);
}

static bool validate_jf_arm_packet(const uint8_t *packet)
{
    const uint16_t crc_calc =
        crc16_ccitt_false(packet, ARM_PACKET_JF_SIZE - sizeof(uint16_t));
    const uint16_t crc_packet =
        (uint16_t)packet[ARM_PACKET_JF_SIZE - 2U] |
        ((uint16_t)packet[ARM_PACKET_JF_SIZE - 1U] << 8);

    return (packet[0] == 'J') && (packet[1] == 'F') &&
           (crc_calc == crc_packet);
}

static bool validate_uf_v2_packet(const uint8_t *packet)
{
    const uint16_t crc_calc =
        crc16_ccitt_false(packet, UF_PACKET_V2_CRC_OFFSET);
    const uint16_t crc_packet =
        (uint16_t)packet[UF_PACKET_V2_CRC_OFFSET] |
        ((uint16_t)packet[UF_PACKET_V2_CRC_OFFSET + 1U] << 8);

    return (packet[0] == 'U') && (packet[1] == 'F') &&
           (packet[UF_PACKET_V2_PAYLOAD_LEN_OFFSET] <=
            UF_PACKET_V2_PAYLOAD_MAX_LEN) &&
           (crc_calc == crc_packet);
}

static uint16_t find_next_xbee_arm_sync_offset(const uint8_t *packet,
                                               uint16_t length,
                                               uint16_t start_offset)
{
    if (length < 2U) {
        return length;
    }

    for (uint16_t i = start_offset; (i + 1U) < length; i++) {
        if ((packet[i] == 'J') && (packet[i + 1U] == 'F')) {
            return i;
        }
    }

    return length;
}

static uint16_t find_next_xbee_uf_sync_offset(const uint8_t *packet,
                                              uint16_t length,
                                              uint16_t start_offset)
{
    if (length < 2U) {
        return length;
    }

    for (uint16_t i = start_offset; (i + 1U) < length; i++) {
        if ((packet[i] == 'U') && (packet[i + 1U] == 'F')) {
            return i;
        }
    }

    return length;
}

static void start_xbee_arm_packet(void)
{
    g_router.xbee_rover_pending_j = false;
    g_router.xbee_rover_pending_j_sync_only = false;
    g_router.xbee_rover_pending_u = false;
    g_router.xbee_rover_pending_u_sync_only = false;
    g_router.xbee_rover_len = 0U;
    g_router.xbee_text_overflow = false;
    g_router.xbee_arm_buf[0] = 'J';
    g_router.xbee_arm_buf[1] = 'F';
    g_router.xbee_arm_len = 2U;
    g_router.xbee_uf_len = 0U;
    g_router.xbee_filter_mode = XBEE_FILTER_ARM;
}

static void start_xbee_uf_packet(void)
{
    g_router.xbee_rover_pending_j = false;
    g_router.xbee_rover_pending_j_sync_only = false;
    g_router.xbee_rover_pending_u = false;
    g_router.xbee_rover_pending_u_sync_only = false;
    g_router.xbee_rover_len = 0U;
    g_router.xbee_text_overflow = false;
    g_router.xbee_arm_len = 0U;
    g_router.xbee_uf_buf[0] = 'U';
    g_router.xbee_uf_buf[1] = 'F';
    g_router.xbee_uf_len = 2U;
    g_router.xbee_filter_mode = XBEE_FILTER_UF;
}

static void discard_invalid_xbee_arm_packet_or_resync(void)
{
    const uint16_t sync_offset =
        find_next_xbee_arm_sync_offset(g_router.xbee_arm_buf,
                                       g_router.xbee_arm_len, 1U);

    LOG("[integrated] arm downlink rejected by crc len=%u\r\n",
        (unsigned int)g_router.xbee_arm_len);
    note_module_downlink_status(LINK_STAT_STATUS_CRC);

    if (sync_offset < g_router.xbee_arm_len) {
        const uint16_t remaining =
            (uint16_t)(g_router.xbee_arm_len - sync_offset);
        memmove(g_router.xbee_arm_buf,
                &g_router.xbee_arm_buf[sync_offset], remaining);
        g_router.xbee_arm_len = remaining;
        g_router.xbee_filter_mode = XBEE_FILTER_ARM;
        return;
    }

    g_router.xbee_rover_pending_j =
        (g_router.xbee_arm_len > 0U) &&
        (g_router.xbee_arm_buf[g_router.xbee_arm_len - 1U] == 'J');
    g_router.xbee_rover_pending_j_sync_only = g_router.xbee_rover_pending_j;
    g_router.xbee_rover_pending_u =
        (g_router.xbee_arm_len > 0U) &&
        (g_router.xbee_arm_buf[g_router.xbee_arm_len - 1U] == 'U');
    g_router.xbee_rover_pending_u_sync_only = g_router.xbee_rover_pending_u;
    g_router.xbee_arm_len = 0U;
    g_router.xbee_filter_mode = XBEE_FILTER_ROVER;
}

static void discard_invalid_xbee_uf_packet_or_resync(link_stat_status_t status)
{
    const uint16_t sync_offset =
        find_next_xbee_uf_sync_offset(g_router.xbee_uf_buf,
                                      g_router.xbee_uf_len, 1U);

    LOG("[integrated] UF v2 downlink rejected len=%u\r\n",
        (unsigned int)g_router.xbee_uf_len);
    note_module_downlink_status(status);

    if (sync_offset < g_router.xbee_uf_len) {
        const uint16_t remaining =
            (uint16_t)(g_router.xbee_uf_len - sync_offset);
        memmove(g_router.xbee_uf_buf,
                &g_router.xbee_uf_buf[sync_offset], remaining);
        g_router.xbee_uf_len = remaining;
        g_router.xbee_filter_mode = XBEE_FILTER_UF;
        return;
    }

    g_router.xbee_rover_pending_j =
        (g_router.xbee_uf_len > 0U) &&
        (g_router.xbee_uf_buf[g_router.xbee_uf_len - 1U] == 'J');
    g_router.xbee_rover_pending_j_sync_only = g_router.xbee_rover_pending_j;
    g_router.xbee_rover_pending_u =
        (g_router.xbee_uf_len > 0U) &&
        (g_router.xbee_uf_buf[g_router.xbee_uf_len - 1U] == 'U');
    g_router.xbee_rover_pending_u_sync_only = g_router.xbee_rover_pending_u;
    g_router.xbee_uf_len = 0U;
    g_router.xbee_filter_mode = XBEE_FILTER_ROVER;
}

static void queue_xbee_arm_if_ready(void)
{
    if (g_router.xbee_arm_len < ARM_PACKET_JF_SIZE) {
        return;
    }

    if (!validate_jf_arm_packet(g_router.xbee_arm_buf)) {
        discard_invalid_xbee_arm_packet_or_resync();
        return;
    }

    route_downlink_arm_packet(g_router.xbee_arm_buf);
    g_router.xbee_arm_len = 0U;
    g_router.xbee_filter_mode = XBEE_FILTER_ROVER;
}

static void queue_xbee_uf_if_ready(void)
{
    if (g_router.xbee_uf_len < UF_PACKET_V2_SIZE) {
        return;
    }

    if (g_router.xbee_uf_buf[UF_PACKET_V2_PAYLOAD_LEN_OFFSET] >
        UF_PACKET_V2_PAYLOAD_MAX_LEN) {
        LOG("[integrated] UF v2 downlink payload_len invalid len=%u\r\n",
            (unsigned int)g_router.xbee_uf_buf[UF_PACKET_V2_PAYLOAD_LEN_OFFSET]);
        discard_invalid_xbee_uf_packet_or_resync(LINK_STAT_STATUS_FORMAT);
        return;
    }

    if (!validate_uf_v2_packet(g_router.xbee_uf_buf)) {
        discard_invalid_xbee_uf_packet_or_resync(LINK_STAT_STATUS_CRC);
        return;
    }

    route_downlink_uf_packet(g_router.xbee_uf_buf);
    g_router.xbee_uf_len = 0U;
    g_router.xbee_filter_mode = XBEE_FILTER_ROVER;
}

static void queue_xbee_rover_if_ready(void)
{
    if (g_router.xbee_rover_len == 0U) {
        return;
    }

    if (mode_control_get_module_mode() == MODULE_MODE_SCIENCE) {
        route_downlink_science_mode_text_packet(g_router.xbee_rover_buf,
                                                g_router.xbee_rover_len);
    } else {
        route_downlink_rover_packet(g_router.xbee_rover_buf,
                                    g_router.xbee_rover_len);
    }

    g_router.xbee_rover_len = 0U;
}

static void filter_xbee_science_payload_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (g_router.xbee_text_overflow) {
            g_router.xbee_rover_len = 0U;
            g_router.xbee_text_overflow = false;
            return;
        }

        queue_xbee_rover_if_ready();
        return;
    }

    if (g_router.xbee_text_overflow) {
        return;
    }

    if (g_router.xbee_rover_len < TEXT_PACKET_MAX_LEN) {
        g_router.xbee_rover_buf[g_router.xbee_rover_len++] = byte;
        return;
    }

    note_science_mode_downlink_status(g_router.xbee_rover_buf,
                                      g_router.xbee_rover_len,
                                      LINK_STAT_STATUS_OVERFLOW);
    g_router.xbee_rover_len = 0U;
    g_router.xbee_text_overflow = true;
}

static void append_rejected_xbee_sync_byte(uint8_t byte, bool sync_only)
{
    if (sync_only) {
        return;
    }

    if ((g_router.xbee_rover_len > 0U) &&
        (g_router.xbee_rover_len < ROVER_PACKET_MAX_LEN)) {
        g_router.xbee_rover_buf[g_router.xbee_rover_len++] = byte;
        return;
    }

    if (g_router.xbee_rover_len > 0U) {
        g_router.xbee_rover_len = 0U;
        note_rover_downlink_status(LINK_STAT_STATUS_OVERFLOW);
    }
}

static void filter_xbee_payload_byte(uint8_t byte)
{
    if (mode_control_get_module_mode() == MODULE_MODE_SCIENCE) {
        filter_xbee_science_payload_byte(byte);
        return;
    }

    if (g_router.xbee_filter_mode == XBEE_FILTER_ARM) {
        g_router.xbee_arm_buf[g_router.xbee_arm_len++] = byte;
        queue_xbee_arm_if_ready();
        return;
    }

    if (g_router.xbee_filter_mode == XBEE_FILTER_UF) {
        g_router.xbee_uf_buf[g_router.xbee_uf_len++] = byte;
        queue_xbee_uf_if_ready();
        return;
    }

    if (g_router.xbee_rover_pending_j) {
        const bool sync_only = g_router.xbee_rover_pending_j_sync_only;

        g_router.xbee_rover_pending_j = false;
        g_router.xbee_rover_pending_j_sync_only = false;

        if (byte == 'F') {
            start_xbee_arm_packet();
            return;
        }

        append_rejected_xbee_sync_byte('J', sync_only);
    }

    if (g_router.xbee_rover_pending_u) {
        const bool sync_only = g_router.xbee_rover_pending_u_sync_only;

        g_router.xbee_rover_pending_u = false;
        g_router.xbee_rover_pending_u_sync_only = false;

        if (byte == 'F') {
            start_xbee_uf_packet();
            return;
        }

        append_rejected_xbee_sync_byte('U', sync_only);
    }

    if (byte == 'J') {
        g_router.xbee_rover_pending_j = true;
        g_router.xbee_rover_pending_j_sync_only = false;
        return;
    }

    if (byte == 'U') {
        g_router.xbee_rover_pending_u = true;
        g_router.xbee_rover_pending_u_sync_only = false;
        return;
    }

    if (byte == '\n') {
        queue_xbee_rover_if_ready();
        return;
    }

    if (byte == '\r') {
        return;
    }

    if ((g_router.xbee_rover_len == 0U) && (byte != '0')) {
        note_downlink_status(LINK_STAT_STATUS_FORMAT);
        return;
    }

    if (g_router.xbee_rover_len < ROVER_PACKET_MAX_LEN) {
        g_router.xbee_rover_buf[g_router.xbee_rover_len++] = byte;
        return;
    }

    g_router.xbee_rover_len = 0U;
    note_rover_downlink_status(LINK_STAT_STATUS_OVERFLOW);
}

static void process_xbee_input_byte(uint8_t byte)
{
    filter_xbee_payload_byte(byte);
}

static void poll_xbee_dma(UART_HandleTypeDef *huart,
                          uint8_t *buffer,
                          uint16_t *position,
                          uint16_t size,
                          bool is_active)
{
    const uint16_t dma_pos = get_dma_position(huart, size);

    while (*position != dma_pos) {
        const uint8_t byte = buffer[*position];

        if (is_active) {
            process_xbee_input_byte(byte);
        }

        (*position)++;
        if (*position >= size) {
            *position = 0U;
        }
    }
}

static void flush_xbee_dma_positions(void)
{
    g_router.xbee3_rx_pos =
        get_dma_position(&EXTERNAL_XBEE_UART, sizeof(g_router.xbee3_rx_dma));
    g_router.xbee6_rx_pos =
        get_dma_position(&ONBOARD_XBEE_UART, sizeof(g_router.xbee6_rx_dma));
}

static void sync_modes(void)
{
    const uint32_t generation = mode_control_get_generation();
    const module_mode_t module_mode = mode_control_get_module_mode();
    const xbee_mode_t xbee_mode = mode_control_get_xbee_mode();

    if (generation == g_router.mode_generation) {
        return;
    }

    if (module_mode != g_router.module_mode) {
        g_router.module_mode = module_mode;
        reset_module_uplink_parser();
        reset_xbee_downlink_parser();
        LOG("[integrated] router module mode -> %s\r\n",
            mode_control_get_module_name());
    }

    if (xbee_mode != g_router.xbee_mode) {
        g_router.xbee_mode = xbee_mode;
        reset_xbee_downlink_parser();
        flush_xbee_dma_positions();
        LOG("[integrated] router xbee mode -> %s\r\n",
            mode_control_get_xbee_name());
    }

    g_router.mode_generation = generation;
}

void data_router_init(void)
{
    memset(&g_router, 0, sizeof(g_router));

    g_router.module_mode = mode_control_get_module_mode();
    g_router.xbee_mode = mode_control_get_xbee_mode();
    g_router.mode_generation = mode_control_get_generation();
    reset_module_uplink_parser();
    reset_xbee_downlink_parser();

    tx_channel_init(&g_router.xbee_tx, NULL);
    tx_channel_init(&g_router.rover_out_tx, &ROVER_OUT_UART);
    tx_channel_init(&g_router.module_out_tx, &MODULE_OUT_UART);

    if (start_circular_reception(&ROVER_IN_UART, g_router.rover_rx_dma,
                                 sizeof(g_router.rover_rx_dma)) != HAL_OK) {
        report_error();
        Error_Handler();
    }

    if (start_circular_reception(&MODULE_IN_UART, g_router.module_rx_dma,
                                 sizeof(g_router.module_rx_dma)) != HAL_OK) {
        report_error();
        Error_Handler();
    }

    if (start_circular_reception(&EXTERNAL_XBEE_UART, g_router.xbee3_rx_dma,
                                 sizeof(g_router.xbee3_rx_dma)) != HAL_OK) {
        report_error();
        Error_Handler();
    }

    if (start_circular_reception(&ONBOARD_XBEE_UART, g_router.xbee6_rx_dma,
                                 sizeof(g_router.xbee6_rx_dma)) != HAL_OK) {
        report_error();
        Error_Handler();
    }

    flush_xbee_dma_positions();
    LOG("[integrated] router initialized\r\n");
}

void data_router_poll(void)
{
    UART_HandleTypeDef *active_xbee_uart;

    sync_modes();
    active_xbee_uart = mode_control_get_active_xbee_uart();

    poll_rover_uplink_dma();
    poll_module_uplink_dma();
    poll_xbee_dma(&EXTERNAL_XBEE_UART, g_router.xbee3_rx_dma,
                  &g_router.xbee3_rx_pos, sizeof(g_router.xbee3_rx_dma),
                  active_xbee_uart == &EXTERNAL_XBEE_UART);
    poll_xbee_dma(&ONBOARD_XBEE_UART, g_router.xbee6_rx_dma,
                  &g_router.xbee6_rx_pos, sizeof(g_router.xbee6_rx_dma),
                  active_xbee_uart == &ONBOARD_XBEE_UART);

    tx_channel_pump(&g_router.xbee_tx, active_xbee_uart);
    tx_channel_pump(&g_router.rover_out_tx, NULL);
    tx_channel_pump(&g_router.module_out_tx, NULL);
}

void data_router_on_uart_rx_complete(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void data_router_on_uart_tx_complete(UART_HandleTypeDef *huart)
{
    if (tx_channel_on_complete(&g_router.xbee_tx, huart)) {
        return;
    }

    if (tx_channel_on_complete(&g_router.rover_out_tx, huart)) {
        return;
    }

    (void)tx_channel_on_complete(&g_router.module_out_tx, huart);
}

void data_router_on_uart_error(UART_HandleTypeDef *huart)
{
    LOG("[integrated] uart error on %s: code=0x%08lX\r\n",
        get_uart_name(huart), (unsigned long)HAL_UART_GetError(huart));
    report_error();

    if (huart == &ROVER_IN_UART) {
        g_router.rover_line_len = 0U;
        g_router.rover_line_overflow = false;
        restart_circular_reception(&ROVER_IN_UART, g_router.rover_rx_dma,
                                   sizeof(g_router.rover_rx_dma),
                                   &g_router.rover_rx_pos);
        return;
    }

    if (huart == &MODULE_IN_UART) {
        reset_module_uplink_parser();
        restart_circular_reception(&MODULE_IN_UART, g_router.module_rx_dma,
                                   sizeof(g_router.module_rx_dma),
                                   &g_router.module_rx_pos);
        return;
    }

    if (huart == &EXTERNAL_XBEE_UART) {
        (void)tx_channel_on_error(&g_router.xbee_tx, huart);
        reset_xbee_downlink_parser();
        restart_circular_reception(&EXTERNAL_XBEE_UART, g_router.xbee3_rx_dma,
                                   sizeof(g_router.xbee3_rx_dma),
                                   &g_router.xbee3_rx_pos);
        return;
    }

    if (huart == &ONBOARD_XBEE_UART) {
        (void)tx_channel_on_error(&g_router.xbee_tx, huart);
        reset_xbee_downlink_parser();
        restart_circular_reception(&ONBOARD_XBEE_UART, g_router.xbee6_rx_dma,
                                   sizeof(g_router.xbee6_rx_dma),
                                   &g_router.xbee6_rx_pos);
        return;
    }

    if (tx_channel_on_error(&g_router.rover_out_tx, huart)) {
        return;
    }

    (void)tx_channel_on_error(&g_router.module_out_tx, huart);
}
