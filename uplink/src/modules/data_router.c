#include "modules/data_router.h"

#include "debug_log.h"
#include "main.h"
#include "modules/output_source_selector.h"
#include "modules/status_leds.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define ROVER_IN_UART                 huart1
#define ARM_IN_UART                   huart2
#define USB_IO_UART                   huart3
#define XBEE_IO_UART                  huart6
#define DOWNLINK_UART                 huart4
#define ROVER_DMA_RX_BUFFER_SIZE      256U
#define ARM_DMA_RX_BUFFER_SIZE        256U
#define BRIDGE_DMA_RX_BUFFER_SIZE     256U
#define UPLINK_TX_QUEUE_DEPTH         16U
#define UPLINK_TX_FRAME_MAX_LEN       128U
#define DOWNLINK_TX_QUEUE_SIZE        1024U
#define TEXT_LINE_BUFFER_SIZE         127U

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

typedef struct
{
    uint8_t data[UPLINK_TX_FRAME_MAX_LEN];
    uint16_t len;
    uint8_t type;
} UplinkTxFrame;

typedef enum
{
    ARM_SYNC_WAIT_A = 0,
    ARM_SYNC_WAIT_C,
    ARM_SYNC_COLLECT_PAYLOAD,
} ArmRxState;

typedef enum
{
    UPLINK_TX_FRAME_TYPE_ROVER = 0,
    UPLINK_TX_FRAME_TYPE_ARM,
    UPLINK_TX_FRAME_TYPE_SCIENCE,
} UplinkTxFrameType;

typedef struct
{
    UART_HandleTypeDef *active_output_uart;
    UART_HandleTypeDef *active_bridge_input_uart;
    bool science_mode_enabled;

    uint8_t rover_dma_rx_buffer[ROVER_DMA_RX_BUFFER_SIZE];
    uint16_t rover_dma_last_pos;
    char rover_line_buffer[TEXT_LINE_BUFFER_SIZE];
    uint16_t rover_line_index;
    bool rover_line_overflow;

    uint8_t arm_dma_rx_buffer[ARM_DMA_RX_BUFFER_SIZE];
    uint16_t arm_dma_last_pos;
    char science_line_buffer[TEXT_LINE_BUFFER_SIZE];
    uint16_t science_line_index;
    bool science_line_overflow;
    uint8_t arm_packet_buffer[sizeof(PacketAC_v6)];
    uint16_t arm_packet_index;
    ArmRxState arm_rx_state;

    UplinkTxFrame uplink_tx_queue[UPLINK_TX_QUEUE_DEPTH];
    volatile uint16_t uplink_tx_head;
    volatile uint16_t uplink_tx_tail;
    volatile uint16_t uplink_tx_count;
    volatile bool uplink_tx_busy;
    UART_HandleTypeDef *tx_uart;
    uint8_t uplink_tx_dma_buffer[UPLINK_TX_FRAME_MAX_LEN];

    uint8_t usb_bridge_dma_rx_buffer[BRIDGE_DMA_RX_BUFFER_SIZE];
    uint16_t usb_bridge_dma_last_pos;
    uint8_t xbee_bridge_dma_rx_buffer[BRIDGE_DMA_RX_BUFFER_SIZE];
    uint16_t xbee_bridge_dma_last_pos;
    uint8_t downlink_tx_queue[DOWNLINK_TX_QUEUE_SIZE];
    volatile uint16_t downlink_tx_head;
    volatile uint16_t downlink_tx_tail;
    volatile uint16_t downlink_tx_count;
    volatile bool downlink_tx_busy;
    uint8_t downlink_tx_byte;
} DataRouterContext;

static DataRouterContext g_data_router;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
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

#if DEBUG_LOG_ENABLED
static const char *get_uart_name(const UART_HandleTypeDef *huart)
{
    if (huart == &ROVER_IN_UART) {
        return "Rover IN (USART1)";
    }

    if (huart == &ARM_IN_UART) {
        return g_data_router.science_mode_enabled ? "Science IN (USART2)"
                                                  : "Arm IN (USART2)";
    }

    if (huart == &USB_IO_UART) {
        return "USB I/O (USART3)";
    }

    if (huart == &XBEE_IO_UART) {
        return "XBee I/O (USART6)";
    }

    if (huart == &DOWNLINK_UART) {
        return "Downlink OUT (UART4)";
    }

    return "unknown";
}
#endif

static bool is_output_uart(const UART_HandleTypeDef *huart)
{
    return (huart == &USB_IO_UART) || (huart == &XBEE_IO_UART);
}

static void note_rx_activity(void)
{
    status_leds_on_rx_activity();
}

static uint16_t get_dma_rx_pos(const UART_HandleTypeDef *huart, uint16_t size)
{
    return (uint16_t)(size - __HAL_DMA_GET_COUNTER(huart->hdmarx));
}

static HAL_StatusTypeDef start_circular_reception(UART_HandleTypeDef *huart,
                                                  uint8_t *buffer,
                                                  uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UART_Receive_DMA(huart, buffer, size);

    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_TC);
    }

    return status;
}

static void restart_circular_reception(UART_HandleTypeDef *huart,
                                       uint8_t *buffer,
                                       uint16_t size,
                                       uint16_t *last_pos)
{
    (void)HAL_UART_DMAStop(huart);
    *last_pos = 0U;

    if (start_circular_reception(huart, buffer, size) != HAL_OK) {
        Error_Handler();
    }
}

static void sync_active_output_uart(bool log_change)
{
    UART_HandleTypeDef *active_uart = uplink_output_source_get_active_uart();
    bool changed = false;
    uint32_t primask = enter_critical_section();

    if (g_data_router.active_output_uart != active_uart) {
        g_data_router.active_output_uart = active_uart;
        changed = true;
    }
    exit_critical_section(primask);

    if (changed && log_change) {
        LOG("[uplink] router output -> %s\r\n",
            uplink_output_source_get_current_name());
    }
}

static void skip_pending_bridge_rx(UART_HandleTypeDef *huart)
{
    if (huart == &USB_IO_UART) {
        g_data_router.usb_bridge_dma_last_pos =
            get_dma_rx_pos(&USB_IO_UART,
                           sizeof(g_data_router.usb_bridge_dma_rx_buffer));
        return;
    }

    if (huart == &XBEE_IO_UART) {
        g_data_router.xbee_bridge_dma_last_pos =
            get_dma_rx_pos(&XBEE_IO_UART,
                           sizeof(g_data_router.xbee_bridge_dma_rx_buffer));
    }
}

static void sync_active_bridge_input_uart(bool log_change)
{
    UART_HandleTypeDef *active_uart = g_data_router.active_output_uart;
    bool changed = false;
    uint32_t primask;

    if (active_uart == NULL) {
        return;
    }

    primask = enter_critical_section();
    if (g_data_router.active_bridge_input_uart != active_uart) {
        g_data_router.active_bridge_input_uart = active_uart;
        changed = true;
    }
    exit_critical_section(primask);

    if (!changed) {
        return;
    }

    skip_pending_bridge_rx(active_uart);

    if (log_change) {
        LOG("[uplink] bridge input -> %s\r\n",
            uplink_output_source_get_current_name());
    }
}

static void reset_text_line_receiver(uint16_t *index, bool *overflow)
{
    *index = 0U;
    *overflow = false;
}

static void sync_science_mode(void)
{
    const bool science_mode_enabled =
        uplink_output_source_is_science_mode_enabled();

    if (g_data_router.science_mode_enabled == science_mode_enabled) {
        return;
    }

    g_data_router.science_mode_enabled = science_mode_enabled;
    reset_text_line_receiver(&g_data_router.rover_line_index,
                             &g_data_router.rover_line_overflow);
    reset_text_line_receiver(&g_data_router.science_line_index,
                             &g_data_router.science_line_overflow);
    g_data_router.arm_packet_index = 0U;
    g_data_router.arm_rx_state = ARM_SYNC_WAIT_A;
    g_data_router.rover_dma_last_pos =
        get_dma_rx_pos(&ROVER_IN_UART, sizeof(g_data_router.rover_dma_rx_buffer));
    g_data_router.arm_dma_last_pos =
        get_dma_rx_pos(&ARM_IN_UART, sizeof(g_data_router.arm_dma_rx_buffer));
}

static bool enqueue_uplink_frame(UplinkTxFrameType type, const uint8_t *data, uint16_t len)
{
    UplinkTxFrame *frame;
    uint32_t primask;

    if ((len == 0U) || (len > UPLINK_TX_FRAME_MAX_LEN)) {
        return false;
    }

    primask = enter_critical_section();
    if (g_data_router.uplink_tx_count >= UPLINK_TX_QUEUE_DEPTH) {
        exit_critical_section(primask);
        return false;
    }

    frame = &g_data_router.uplink_tx_queue[g_data_router.uplink_tx_tail];
    frame->type = (uint8_t)type;
    memcpy(frame->data, data, len);
    frame->len = len;

    g_data_router.uplink_tx_tail =
        (uint16_t)((g_data_router.uplink_tx_tail + 1U) % UPLINK_TX_QUEUE_DEPTH);
    g_data_router.uplink_tx_count++;
    exit_critical_section(primask);
    return true;
}

#if DEBUG_LOG_ENABLED
static const char *get_text_frame_type_name(UplinkTxFrameType type)
{
    return (type == UPLINK_TX_FRAME_TYPE_SCIENCE) ? "science" : "rover";
}

static void log_tx_start(UplinkTxFrameType type,
                         const UART_HandleTypeDef *output_uart,
                         const uint8_t *data,
                         uint16_t len)
{
    if ((type == UPLINK_TX_FRAME_TYPE_ROVER) ||
        (type == UPLINK_TX_FRAME_TYPE_SCIENCE)) {
        char rover_line[TEXT_LINE_BUFFER_SIZE];
        uint16_t text_len = len;

        if ((text_len >= 2U) && (data[text_len - 2U] == '\r') &&
            (data[text_len - 1U] == '\n')) {
            text_len -= 2U;
        }

        if (text_len >= sizeof(rover_line)) {
            text_len = (uint16_t)(sizeof(rover_line) - 1U);
        }

        memcpy(rover_line, data, text_len);
        rover_line[text_len] = '\0';

        LOG("[uplink] tx %s -> %s: \"%s\"\r\n",
            get_text_frame_type_name(type),
            get_uart_name(output_uart),
            rover_line);
        return;
    }

    if ((type == UPLINK_TX_FRAME_TYPE_ARM) && (len == sizeof(PacketAC_v6))) {
        LOG("[uplink] tx arm -> %s:", get_uart_name(output_uart));
        for (uint16_t i = 0U; i < len; i++) {
            LOG(" %02X", data[i]);
        }
        LOG("\r\n");
        return;
    }

    LOG("[uplink] tx raw -> %s: len=%u\r\n",
        get_uart_name(output_uart),
        (unsigned int)len);
}
#endif

static void pump_uplink_tx(void)
{
    UART_HandleTypeDef *output_uart;
    uint16_t frame_len;
#if DEBUG_LOG_ENABLED
    UplinkTxFrameType frame_type;
#endif
    uint32_t primask = enter_critical_section();

    if (g_data_router.uplink_tx_busy || (g_data_router.uplink_tx_count == 0U)) {
        exit_critical_section(primask);
        return;
    }

    output_uart = g_data_router.active_output_uart;
    if (output_uart == NULL) {
        exit_critical_section(primask);
        return;
    }

    frame_len = g_data_router.uplink_tx_queue[g_data_router.uplink_tx_head].len;
#if DEBUG_LOG_ENABLED
    frame_type =
        (UplinkTxFrameType)g_data_router.uplink_tx_queue[g_data_router.uplink_tx_head].type;
#endif
    memcpy(g_data_router.uplink_tx_dma_buffer,
           g_data_router.uplink_tx_queue[g_data_router.uplink_tx_head].data,
           frame_len);
    g_data_router.uplink_tx_busy = true;
    g_data_router.tx_uart = output_uart;
    exit_critical_section(primask);

#if DEBUG_LOG_ENABLED
    log_tx_start(frame_type, output_uart, g_data_router.uplink_tx_dma_buffer, frame_len);
#endif

    if (HAL_UART_Transmit_DMA(output_uart, g_data_router.uplink_tx_dma_buffer,
                              frame_len) != HAL_OK) {
        primask = enter_critical_section();
        g_data_router.uplink_tx_busy = false;
        g_data_router.tx_uart = NULL;
        exit_critical_section(primask);
    }
}

static bool enqueue_downlink_byte(uint8_t byte)
{
    uint32_t primask = enter_critical_section();

    if (g_data_router.downlink_tx_count >= DOWNLINK_TX_QUEUE_SIZE) {
        exit_critical_section(primask);
        return false;
    }

    g_data_router.downlink_tx_queue[g_data_router.downlink_tx_tail] = byte;
    g_data_router.downlink_tx_tail =
        (uint16_t)((g_data_router.downlink_tx_tail + 1U) %
                   DOWNLINK_TX_QUEUE_SIZE);
    g_data_router.downlink_tx_count++;
    exit_critical_section(primask);
    return true;
}

static void pump_downlink_tx(void)
{
    HAL_StatusTypeDef status;
    uint32_t primask = enter_critical_section();

    if (g_data_router.downlink_tx_busy ||
        (g_data_router.downlink_tx_count == 0U)) {
        exit_critical_section(primask);
        return;
    }

    g_data_router.downlink_tx_byte =
        g_data_router.downlink_tx_queue[g_data_router.downlink_tx_head];
    g_data_router.downlink_tx_busy = true;
    exit_critical_section(primask);

    status = HAL_UART_Transmit_IT(&DOWNLINK_UART,
                                  &g_data_router.downlink_tx_byte,
                                  1U);
    if (status != HAL_OK) {
        primask = enter_critical_section();
        g_data_router.downlink_tx_busy = false;
        exit_critical_section(primask);
    }
}

static bool is_hex_digit(char c)
{
    return ((c >= '0') && (c <= '9')) ||
           ((c >= 'A') && (c <= 'F')) ||
           ((c >= 'a') && (c <= 'f'));
}

static unsigned long hex_digit_to_value(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return (unsigned long)(c - '0');
    }

    if ((c >= 'A') && (c <= 'F')) {
        return (unsigned long)(c - 'A' + 10);
    }

    return (unsigned long)(c - 'a' + 10);
}

static bool validate_rover_line(const char *line)
{
    const char *cursor = line;
    unsigned long can_id = 0UL;

    if ((cursor[0] != '0') || ((cursor[1] != 'x') && (cursor[1] != 'X'))) {
        return false;
    }

    cursor += 2;
    if (*cursor == '\0') {
        return false;
    }

    if ((*cursor != '3') && (*cursor != '4')) {
        return false;
    }

    while ((*cursor != '\0') && (*cursor != ',')) {
        const char c = *cursor;

        if (!is_hex_digit(c)) {
            return false;
        }

        can_id = (can_id * 16UL) + hex_digit_to_value(c);
        cursor++;
    }

    if ((*cursor != ',') || (can_id > 0x7FFUL)) {
        return false;
    }

    cursor++;
    if (*cursor == '\0') {
        return false;
    }

    return true;
}

static bool validate_science_mode_text_line(const char *line, char first_id_digit)
{
    const char *payload = &line[6];

    if ((line[0] != '0') || ((line[1] != 'x') && (line[1] != 'X'))) {
        return false;
    }

    if (line[2] != first_id_digit) {
        return false;
    }

    if (!is_hex_digit(line[3]) || !is_hex_digit(line[4]) ||
        (line[5] != ',')) {
        return false;
    }

    if (*payload == '\0') {
        return false;
    }

    return true;
}

static bool validate_science_mode_rover_line(const char *line)
{
    return validate_science_mode_text_line(line, '3') ||
           validate_science_mode_text_line(line, '4');
}

static bool validate_science_line(const char *line)
{
    return validate_science_mode_text_line(line, '5');
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0; i < len; i++) {
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

static bool validate_arm_packet(const uint8_t *raw_packet)
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

static void enqueue_text_frame(UplinkTxFrameType type, const char *line, uint16_t line_len)
{
    uint8_t tx_frame[UPLINK_TX_FRAME_MAX_LEN];

    memcpy(tx_frame, line, line_len);
    tx_frame[line_len] = '\r';
    tx_frame[line_len + 1U] = '\n';
    if (!enqueue_uplink_frame(type, tx_frame, (uint16_t)(line_len + 2U))) {
#if DEBUG_LOG_ENABLED
        LOG("[uplink] %s tx queue full\r\n", get_text_frame_type_name(type));
#endif
    }
}

static void consume_text_line_byte(uint8_t byte,
                                   char *line_buffer,
                                   uint16_t *line_index,
                                   bool *line_overflow,
                                   bool (*validator)(const char *),
                                   UplinkTxFrameType frame_type)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        if (*line_overflow) {
            reset_text_line_receiver(line_index, line_overflow);
            return;
        }

        if (*line_index == 0U) {
            return;
        }

        line_buffer[*line_index] = '\0';
        if (validator(line_buffer)) {
            enqueue_text_frame(frame_type, line_buffer, *line_index);
        }

        *line_index = 0U;
        return;
    }

    if (*line_overflow) {
        return;
    }

    if (*line_index >= (TEXT_LINE_BUFFER_SIZE - 1U)) {
        reset_text_line_receiver(line_index, line_overflow);
        *line_overflow = true;
        return;
    }

    line_buffer[(*line_index)++] = (char)byte;
}

static void consume_rover_byte(uint8_t byte)
{
    consume_text_line_byte(byte, g_data_router.rover_line_buffer,
                           &g_data_router.rover_line_index,
                           &g_data_router.rover_line_overflow,
                           validate_rover_line, UPLINK_TX_FRAME_TYPE_ROVER);
}

static void consume_science_mode_rover_byte(uint8_t byte)
{
    consume_text_line_byte(byte, g_data_router.rover_line_buffer,
                           &g_data_router.rover_line_index,
                           &g_data_router.rover_line_overflow,
                           validate_science_mode_rover_line,
                           UPLINK_TX_FRAME_TYPE_ROVER);
}

static void consume_science_byte(uint8_t byte)
{
    consume_text_line_byte(byte, g_data_router.science_line_buffer,
                           &g_data_router.science_line_index,
                           &g_data_router.science_line_overflow,
                           validate_science_line,
                           UPLINK_TX_FRAME_TYPE_SCIENCE);
}

static void consume_arm_byte(uint8_t byte)
{
    switch (g_data_router.arm_rx_state) {
    case ARM_SYNC_WAIT_A:
        if (byte == 'A') {
            g_data_router.arm_packet_buffer[0] = byte;
            g_data_router.arm_packet_index = 1U;
            g_data_router.arm_rx_state = ARM_SYNC_WAIT_C;
        }
        break;

    case ARM_SYNC_WAIT_C:
        if (byte == 'C') {
            g_data_router.arm_packet_buffer[1] = byte;
            g_data_router.arm_packet_index = 2U;
            g_data_router.arm_rx_state = ARM_SYNC_COLLECT_PAYLOAD;
        } else if (byte == 'A') {
            g_data_router.arm_packet_buffer[0] = byte;
            g_data_router.arm_packet_index = 1U;
        } else {
            g_data_router.arm_packet_index = 0U;
            g_data_router.arm_rx_state = ARM_SYNC_WAIT_A;
        }
        break;

    case ARM_SYNC_COLLECT_PAYLOAD:
        g_data_router.arm_packet_buffer[g_data_router.arm_packet_index++] = byte;
        if (g_data_router.arm_packet_index >= sizeof(PacketAC_v6)) {
            if (validate_arm_packet(g_data_router.arm_packet_buffer)) {
                if (!enqueue_uplink_frame(UPLINK_TX_FRAME_TYPE_ARM,
                                          g_data_router.arm_packet_buffer,
                                          sizeof(PacketAC_v6))) {
                    LOG("[uplink] arm tx queue full\r\n");
                }
            }

            g_data_router.arm_packet_index = 0U;
            g_data_router.arm_rx_state = ARM_SYNC_WAIT_A;
        }
        break;

    default:
        g_data_router.arm_packet_index = 0U;
        g_data_router.arm_rx_state = ARM_SYNC_WAIT_A;
        break;
    }
}

static void consume_bridge_byte(UART_HandleTypeDef *source_uart, uint8_t byte)
{
    note_rx_activity();

    if (!enqueue_downlink_byte(byte)) {
#if DEBUG_LOG_ENABLED
        LOG("[uplink] downlink tx queue full; dropped byte from %s\r\n",
            get_uart_name(source_uart));
#else
        (void)source_uart;
#endif
    }
}

static void poll_rover_dma_rx(void)
{
    const uint16_t dma_pos =
        get_dma_rx_pos(&ROVER_IN_UART, sizeof(g_data_router.rover_dma_rx_buffer));

    while (g_data_router.rover_dma_last_pos != dma_pos) {
        const uint8_t byte =
            g_data_router.rover_dma_rx_buffer[g_data_router.rover_dma_last_pos];

        note_rx_activity();
        if (g_data_router.science_mode_enabled) {
            consume_science_mode_rover_byte(byte);
        } else {
            consume_rover_byte(byte);
        }

        g_data_router.rover_dma_last_pos++;
        if (g_data_router.rover_dma_last_pos >=
            sizeof(g_data_router.rover_dma_rx_buffer)) {
            g_data_router.rover_dma_last_pos = 0U;
        }
    }
}

static void poll_arm_dma_rx(void)
{
    const uint16_t dma_pos =
        get_dma_rx_pos(&ARM_IN_UART, sizeof(g_data_router.arm_dma_rx_buffer));

    while (g_data_router.arm_dma_last_pos != dma_pos) {
        const uint8_t byte =
            g_data_router.arm_dma_rx_buffer[g_data_router.arm_dma_last_pos];

        note_rx_activity();
        consume_arm_byte(byte);

        g_data_router.arm_dma_last_pos++;
        if (g_data_router.arm_dma_last_pos >= sizeof(g_data_router.arm_dma_rx_buffer)) {
            g_data_router.arm_dma_last_pos = 0U;
        }
    }
}

static void poll_science_dma_rx(void)
{
    const uint16_t dma_pos =
        get_dma_rx_pos(&ARM_IN_UART, sizeof(g_data_router.arm_dma_rx_buffer));

    while (g_data_router.arm_dma_last_pos != dma_pos) {
        const uint8_t byte =
            g_data_router.arm_dma_rx_buffer[g_data_router.arm_dma_last_pos];

        note_rx_activity();
        consume_science_byte(byte);

        g_data_router.arm_dma_last_pos++;
        if (g_data_router.arm_dma_last_pos >= sizeof(g_data_router.arm_dma_rx_buffer)) {
            g_data_router.arm_dma_last_pos = 0U;
        }
    }
}

static void poll_bridge_dma_rx(UART_HandleTypeDef *huart,
                               uint8_t *buffer,
                               uint16_t buffer_size,
                               uint16_t *last_pos)
{
    const uint16_t dma_pos = get_dma_rx_pos(huart, buffer_size);

    while (*last_pos != dma_pos) {
        consume_bridge_byte(huart, buffer[*last_pos]);

        (*last_pos)++;
        if (*last_pos >= buffer_size) {
            *last_pos = 0U;
        }
    }
}

static void restart_bridge_reception(UART_HandleTypeDef *huart)
{
    if (huart == &USB_IO_UART) {
        restart_circular_reception(&USB_IO_UART,
                                   g_data_router.usb_bridge_dma_rx_buffer,
                                   sizeof(g_data_router.usb_bridge_dma_rx_buffer),
                                   &g_data_router.usb_bridge_dma_last_pos);
        return;
    }

    if (huart == &XBEE_IO_UART) {
        restart_circular_reception(&XBEE_IO_UART,
                                   g_data_router.xbee_bridge_dma_rx_buffer,
                                   sizeof(g_data_router.xbee_bridge_dma_rx_buffer),
                                   &g_data_router.xbee_bridge_dma_last_pos);
    }
}

void data_router_init(void)
{
    memset(&g_data_router, 0, sizeof(g_data_router));
    g_data_router.arm_rx_state = ARM_SYNC_WAIT_A;
    g_data_router.science_mode_enabled =
        uplink_output_source_is_science_mode_enabled();

    if (start_circular_reception(&ROVER_IN_UART, g_data_router.rover_dma_rx_buffer,
                                 sizeof(g_data_router.rover_dma_rx_buffer)) != HAL_OK) {
        Error_Handler();
    }

    if (start_circular_reception(&ARM_IN_UART, g_data_router.arm_dma_rx_buffer,
                                 sizeof(g_data_router.arm_dma_rx_buffer)) != HAL_OK) {
        Error_Handler();
    }

    if (start_circular_reception(&USB_IO_UART,
                                 g_data_router.usb_bridge_dma_rx_buffer,
                                 sizeof(g_data_router.usb_bridge_dma_rx_buffer)) != HAL_OK) {
        Error_Handler();
    }

    if (start_circular_reception(&XBEE_IO_UART,
                                 g_data_router.xbee_bridge_dma_rx_buffer,
                                 sizeof(g_data_router.xbee_bridge_dma_rx_buffer)) != HAL_OK) {
        Error_Handler();
    }

    sync_active_output_uart(true);
    sync_active_bridge_input_uart(true);

    LOG("[uplink] bridge USART3/USART6 RX -> UART4 TX\r\n");
}

void data_router_poll(void)
{
    sync_active_output_uart(true);
    sync_active_bridge_input_uart(true);
    sync_science_mode();
    poll_rover_dma_rx();
    if (g_data_router.science_mode_enabled) {
        poll_science_dma_rx();
    } else {
        poll_arm_dma_rx();
    }
    if (g_data_router.active_bridge_input_uart == &USB_IO_UART) {
        poll_bridge_dma_rx(&USB_IO_UART,
                           g_data_router.usb_bridge_dma_rx_buffer,
                           sizeof(g_data_router.usb_bridge_dma_rx_buffer),
                           &g_data_router.usb_bridge_dma_last_pos);
    } else if (g_data_router.active_bridge_input_uart == &XBEE_IO_UART) {
        poll_bridge_dma_rx(&XBEE_IO_UART,
                           g_data_router.xbee_bridge_dma_rx_buffer,
                           sizeof(g_data_router.xbee_bridge_dma_rx_buffer),
                           &g_data_router.xbee_bridge_dma_last_pos);
    }
    pump_uplink_tx();
    pump_downlink_tx();
}

void data_router_on_uart_rx_complete(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void data_router_on_uart_tx_complete(UART_HandleTypeDef *huart)
{
    uint32_t primask;

    if (huart == &DOWNLINK_UART) {
        primask = enter_critical_section();
        if (g_data_router.downlink_tx_count > 0U) {
            g_data_router.downlink_tx_head =
                (uint16_t)((g_data_router.downlink_tx_head + 1U) %
                           DOWNLINK_TX_QUEUE_SIZE);
            g_data_router.downlink_tx_count--;
        }
        g_data_router.downlink_tx_busy = false;
        exit_critical_section(primask);

        pump_downlink_tx();
        return;
    }

    if (huart != g_data_router.tx_uart) {
        return;
    }

    primask = enter_critical_section();
    if (g_data_router.uplink_tx_count > 0U) {
        g_data_router.uplink_tx_head =
            (uint16_t)((g_data_router.uplink_tx_head + 1U) % UPLINK_TX_QUEUE_DEPTH);
        g_data_router.uplink_tx_count--;
    }
    g_data_router.uplink_tx_busy = false;
    g_data_router.tx_uart = NULL;
    exit_critical_section(primask);

    pump_uplink_tx();
}

void data_router_on_uart_error(UART_HandleTypeDef *huart)
{
    uint32_t primask;

#if DEBUG_LOG_ENABLED
    LOG("[uplink] uart error on %s: code=0x%08lX\r\n",
        get_uart_name(huart),
        (unsigned long)HAL_UART_GetError(huart));
#endif

    if (huart == &ROVER_IN_UART) {
        reset_text_line_receiver(&g_data_router.rover_line_index,
                                 &g_data_router.rover_line_overflow);
        restart_circular_reception(&ROVER_IN_UART, g_data_router.rover_dma_rx_buffer,
                                   sizeof(g_data_router.rover_dma_rx_buffer),
                                   &g_data_router.rover_dma_last_pos);
        return;
    }

    if (huart == &ARM_IN_UART) {
        reset_text_line_receiver(&g_data_router.science_line_index,
                                 &g_data_router.science_line_overflow);
        g_data_router.arm_packet_index = 0U;
        g_data_router.arm_rx_state = ARM_SYNC_WAIT_A;
        restart_circular_reception(&ARM_IN_UART, g_data_router.arm_dma_rx_buffer,
                                   sizeof(g_data_router.arm_dma_rx_buffer),
                                   &g_data_router.arm_dma_last_pos);
        return;
    }

    if (is_output_uart(huart)) {
        restart_bridge_reception(huart);

        primask = enter_critical_section();
        if (huart == g_data_router.tx_uart) {
            g_data_router.uplink_tx_busy = false;
            g_data_router.tx_uart = NULL;
        }
        exit_critical_section(primask);
        pump_uplink_tx();
        return;
    }

    if (huart != &DOWNLINK_UART) {
        return;
    }

    (void)HAL_UART_AbortTransmit(&DOWNLINK_UART);

    primask = enter_critical_section();
    if (g_data_router.downlink_tx_busy &&
        (g_data_router.downlink_tx_count > 0U)) {
        g_data_router.downlink_tx_head =
            (uint16_t)((g_data_router.downlink_tx_head + 1U) %
                       DOWNLINK_TX_QUEUE_SIZE);
        g_data_router.downlink_tx_count--;
    }
    g_data_router.downlink_tx_busy = false;
    exit_critical_section(primask);

    pump_downlink_tx();
}
