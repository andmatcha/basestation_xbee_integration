#include "modules/data_router.h"

#include "debug_log.h"
#include "main.h"
#include "modules/input_source_selector.h"
#include "modules/status_leds.h"

#include <stdbool.h>
#include <string.h>

#define ROVER_OUT_UART      huart1
#define ARM_OUT_UART        huart2
#define USB_IN_UART         huart3
#define XBEE_IN_UART        huart6
#define ROVER_PACKET_MAX_LEN 64U
#define ARM_PACKET_JF_SIZE   16U
#define ROVER_LOG_MAX_LEN    (ROVER_PACKET_MAX_LEN * 4U + 1U)
#define XBEE_API_FRAME_MAX_LEN 128U
#define XBEE_DIAG_SAMPLE_MAX_LEN 48U
#define XBEE_DIAG_REPORT_AFTER_BYTES 128U

typedef enum
{
    INPUT_MODE_ROVER = 0,
    INPUT_MODE_ARM,
} InputMode;

typedef enum
{
    XBEE_STREAM_MODE_UNKNOWN = 0,
    XBEE_STREAM_MODE_TRANSPARENT,
    XBEE_STREAM_MODE_API,
} XBeeStreamMode;

typedef enum
{
    XBEE_BYTE_DECODE_MODE_RAW = 0,
    XBEE_BYTE_DECODE_MODE_MASK7,
    XBEE_BYTE_DECODE_MODE_INVERTED,
} XBeeByteDecodeMode;

typedef enum
{
    UART_TRACE_SOURCE_NONE = 0,
    UART_TRACE_SOURCE_USART6_IRQ,
    UART_TRACE_SOURCE_DMA2_STREAM1_IRQ,
} UartTraceSource;

typedef struct
{
    bool valid;
    uint32_t sr;
    uint32_t cr1;
    uint32_t cr3;
    HAL_UART_StateTypeDef rx_state;
    uint16_t rx_xfer_count;
} UartIrqSnapshot;

typedef struct
{
    UART_HandleTypeDef *active_input_uart;
    InputMode input_mode;
    bool rover_pending_j;
    uint8_t usb_rx_char;
    uint8_t xbee_rx_char;
    uint8_t rover_rx_buf[ROVER_PACKET_MAX_LEN];
    uint16_t rover_rx_idx;
    uint8_t arm_packet_rx_buf[ARM_PACKET_JF_SIZE];
    uint16_t arm_packet_rx_idx;
    volatile bool rover_packet_pending;
    volatile uint16_t rover_packet_len;
    uint8_t rover_packet_buf[ROVER_PACKET_MAX_LEN];
    volatile bool arm_packet_pending;
    uint8_t arm_packet_buf[ARM_PACKET_JF_SIZE];
    XBeeStreamMode xbee_stream_mode;
    XBeeByteDecodeMode xbee_byte_decode_mode;
    bool xbee_api_in_frame;
    bool xbee_api_escaped;
    uint8_t xbee_api_length_bytes_received;
    uint16_t xbee_api_expected_len;
    uint16_t xbee_api_received_len;
    uint8_t xbee_api_frame_buf[XBEE_API_FRAME_MAX_LEN];
    bool xbee_diag_reported;
    uint16_t xbee_diag_total_bytes;
    uint16_t xbee_diag_msb_set_bytes;
    uint16_t xbee_diag_raw_printable_bytes;
    uint16_t xbee_diag_mask7_printable_bytes;
    uint16_t xbee_diag_inverted_printable_bytes;
    uint16_t xbee_diag_raw_7e_bytes;
    uint16_t xbee_diag_mask7_7e_bytes;
    uint16_t xbee_diag_inverted_7e_bytes;
    uint16_t xbee_diag_raw_jf_pairs;
    uint16_t xbee_diag_mask7_jf_pairs;
    uint16_t xbee_diag_inverted_jf_pairs;
    bool xbee_diag_has_prev_raw;
    bool xbee_diag_has_prev_mask7;
    bool xbee_diag_has_prev_inverted;
    uint8_t xbee_diag_prev_raw;
    uint8_t xbee_diag_prev_mask7;
    uint8_t xbee_diag_prev_inverted;
    uint8_t xbee_diag_sample_len;
    uint8_t xbee_diag_sample_buf[XBEE_DIAG_SAMPLE_MAX_LEN];
} DataRouterContext;

static DataRouterContext g_data_router;
static volatile UartTraceSource g_uart_trace_source;
static volatile UartIrqSnapshot g_usart6_irq_snapshot;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
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

static uint8_t *get_rx_char_slot(UART_HandleTypeDef *huart)
{
    if (huart == &USB_IN_UART) {
        return &g_data_router.usb_rx_char;
    }

    if (huart == &XBEE_IN_UART) {
        return &g_data_router.xbee_rx_char;
    }

    return NULL;
}

static const char *get_input_uart_name(const UART_HandleTypeDef *huart)
{
    return (huart == &USB_IN_UART) ? "USB IN (USART3)" : "XBee IN (USART6)";
}

static const char *get_uart_trace_source_name(void)
{
    switch (g_uart_trace_source) {
    case UART_TRACE_SOURCE_USART6_IRQ:
        return "USART6_IRQ";
    case UART_TRACE_SOURCE_DMA2_STREAM1_IRQ:
        return "DMA2_Stream1_IRQ";
    default:
        return "unknown";
    }
}

static void log_uart_status_flags(uint32_t status, const char *prefix)
{
    if ((status & USART_SR_PE) != 0U) {
        LOG(" %sPE", prefix);
    }
    if ((status & USART_SR_NE) != 0U) {
        LOG(" %sNE", prefix);
    }
    if ((status & USART_SR_FE) != 0U) {
        LOG(" %sFE", prefix);
    }
    if ((status & USART_SR_ORE) != 0U) {
        LOG(" %sORE", prefix);
    }
    if ((status & USART_SR_RXNE) != 0U) {
        LOG(" %sRXNE", prefix);
    }
    if ((status & USART_SR_IDLE) != 0U) {
        LOG(" %sIDLE", prefix);
    }
    if ((status & USART_SR_LBD) != 0U) {
        LOG(" %sLBD", prefix);
    }
    if ((status & USART_SR_TC) != 0U) {
        LOG(" %sTC", prefix);
    }
    if ((status & USART_SR_TXE) != 0U) {
        LOG(" %sTXE", prefix);
    }
}

static void log_uart_error_details(UART_HandleTypeDef *huart)
{
    const uint32_t error = HAL_UART_GetError(huart);
    const uint32_t status = huart->Instance->SR;
    const uint32_t cr1 = huart->Instance->CR1;
    const uint32_t cr3 = huart->Instance->CR3;
    const DMA_HandleTypeDef *hdmarx = huart->hdmarx;
    const bool has_pre_irq_snapshot =
        (huart == &XBEE_IN_UART) && g_usart6_irq_snapshot.valid;

    LOG("[downlink] uart error on %s: err=0x%08lX",
        get_input_uart_name(huart), (unsigned long)error);

    if (error == HAL_UART_ERROR_NONE) {
        LOG(" NONE");
    }
    if ((error & HAL_UART_ERROR_PE) != 0U) {
        LOG(" PE");
    }
    if ((error & HAL_UART_ERROR_NE) != 0U) {
        LOG(" NE");
    }
    if ((error & HAL_UART_ERROR_FE) != 0U) {
        LOG(" FE");
    }
    if ((error & HAL_UART_ERROR_ORE) != 0U) {
        LOG(" ORE");
    }
    if ((error & HAL_UART_ERROR_DMA) != 0U) {
        LOG(" DMA");
    }

    LOG(" sr=0x%08lX", (unsigned long)status);
    log_uart_status_flags(status, "SR_");

    if (has_pre_irq_snapshot) {
        LOG(" pre_sr=0x%08lX",
            (unsigned long)g_usart6_irq_snapshot.sr);
        log_uart_status_flags(g_usart6_irq_snapshot.sr, "PRE_");
    }

    LOG(" selected=%u irq=%s cr1=0x%08lX cr3=0x%08lX rx_state=%lu",
        downlink_input_source_is_selected_uart(huart) ? 1U : 0U,
        get_uart_trace_source_name(),
        (unsigned long)cr1,
        (unsigned long)cr3,
        (unsigned long)huart->RxState);

    if (hdmarx != NULL) {
        LOG(" dma_state=%lu dma_err=0x%08lX dma_cr=0x%08lX dma_ndtr=%lu dma_fcr=0x%08lX",
            (unsigned long)hdmarx->State,
            (unsigned long)hdmarx->ErrorCode,
            (unsigned long)hdmarx->Instance->CR,
            (unsigned long)hdmarx->Instance->NDTR,
            (unsigned long)hdmarx->Instance->FCR);
    }

    if (has_pre_irq_snapshot) {
        LOG(" pre_cr1=0x%08lX pre_cr3=0x%08lX pre_rx_state=%lu pre_rx_count=%u",
            (unsigned long)g_usart6_irq_snapshot.cr1,
            (unsigned long)g_usart6_irq_snapshot.cr3,
            (unsigned long)g_usart6_irq_snapshot.rx_state,
            (unsigned int)g_usart6_irq_snapshot.rx_xfer_count);
    }

    LOG("\r\n");
}

static uint32_t get_pre_irq_error_flags(const UART_HandleTypeDef *huart)
{
    if ((huart == &XBEE_IN_UART) && g_usart6_irq_snapshot.valid) {
        return g_usart6_irq_snapshot.sr &
               (USART_SR_PE | USART_SR_NE | USART_SR_FE | USART_SR_ORE);
    }

    return huart->Instance->SR & (USART_SR_PE | USART_SR_NE | USART_SR_FE | USART_SR_ORE);
}

static bool is_nonblocking_line_error(uint32_t pre_irq_error_flags)
{
    if ((pre_irq_error_flags & (USART_SR_PE | USART_SR_ORE)) != 0U) {
        return false;
    }

    return (pre_irq_error_flags & (USART_SR_NE | USART_SR_FE)) != 0U;
}

static void clear_receive_flags(UART_HandleTypeDef *huart)
{
    if (get_rx_char_slot(huart) == NULL) {
        return;
    }

    __HAL_UART_CLEAR_PEFLAG(huart);
}

static void stop_receive_it(UART_HandleTypeDef *huart)
{
    if (get_rx_char_slot(huart) == NULL) {
        return;
    }

    if (HAL_UART_AbortReceive(huart) != HAL_OK) {
        Error_Handler();
    }

    clear_receive_flags(huart);
}

static void restart_receive_it(UART_HandleTypeDef *huart)
{
    uint8_t *rx_char = get_rx_char_slot(huart);
    HAL_StatusTypeDef status;

    if (rx_char == NULL) {
        return;
    }

    clear_receive_flags(huart);

    status = HAL_UART_Receive_IT(huart, rx_char, 1U);
    if (status == HAL_BUSY) {
        stop_receive_it(huart);
        status = HAL_UART_Receive_IT(huart, rx_char, 1U);
    }

    if (status != HAL_OK) {
        Error_Handler();
    }
}

static const char *get_xbee_byte_decode_mode_name(XBeeByteDecodeMode mode)
{
    switch (mode) {
    case XBEE_BYTE_DECODE_MODE_MASK7:
        return "mask7";
    case XBEE_BYTE_DECODE_MODE_INVERTED:
        return "inverted";
    case XBEE_BYTE_DECODE_MODE_RAW:
    default:
        return "raw";
    }
}

static uint8_t decode_xbee_stream_byte(uint8_t raw_byte)
{
    switch (g_data_router.xbee_byte_decode_mode) {
    case XBEE_BYTE_DECODE_MODE_MASK7:
        return raw_byte & 0x7FU;
    case XBEE_BYTE_DECODE_MODE_INVERTED:
        return (uint8_t)(~raw_byte);
    case XBEE_BYTE_DECODE_MODE_RAW:
    default:
        return raw_byte;
    }
}

static bool is_printable_stream_byte(uint8_t byte)
{
    return (byte == '\t') || (byte == '\r') || (byte == '\n') ||
           ((byte >= 0x20U) && (byte <= 0x7EU));
}

static uint16_t format_escaped_bytes(const uint8_t *bytes, uint16_t length,
                                     char *output, uint16_t output_capacity)
{
    static const char hex[] = "0123456789ABCDEF";
    uint16_t output_idx = 0U;

    if (output_capacity == 0U) {
        return 0U;
    }

    for (uint16_t i = 0U; i < length; i++) {
        const uint8_t byte = bytes[i];

        if ((byte >= 0x20U) && (byte <= 0x7EU) && (byte != '\\') && (byte != '"')) {
            if ((output_idx + 1U) >= output_capacity) {
                break;
            }
            output[output_idx++] = (char)byte;
            continue;
        }

        if ((byte == '\\') || (byte == '"')) {
            if ((output_idx + 2U) >= output_capacity) {
                break;
            }
            output[output_idx++] = '\\';
            output[output_idx++] = (char)byte;
            continue;
        }

        if (byte == '\r') {
            if ((output_idx + 2U) >= output_capacity) {
                break;
            }
            output[output_idx++] = '\\';
            output[output_idx++] = 'r';
            continue;
        }

        if (byte == '\n') {
            if ((output_idx + 2U) >= output_capacity) {
                break;
            }
            output[output_idx++] = '\\';
            output[output_idx++] = 'n';
            continue;
        }

        if (byte == '\t') {
            if ((output_idx + 2U) >= output_capacity) {
                break;
            }
            output[output_idx++] = '\\';
            output[output_idx++] = 't';
            continue;
        }

        if ((output_idx + 4U) >= output_capacity) {
            break;
        }
        output[output_idx++] = '\\';
        output[output_idx++] = 'x';
        output[output_idx++] = hex[(byte >> 4) & 0x0FU];
        output[output_idx++] = hex[byte & 0x0FU];
    }

    output[output_idx] = '\0';
    return output_idx;
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

static bool validate_rover_packet(const uint8_t *packet, uint16_t length)
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
    if ((idx < length) && ((packet[idx] == '-') || (packet[idx] == '+'))) {
        idx++;
    }

    if (idx >= length) {
        return false;
    }

    while (idx < length) {
        if ((packet[idx] < '0') || (packet[idx] > '9')) {
            return false;
        }

        idx++;
    }

    return true;
}

static void update_jf_pair_counter(uint8_t byte, bool *has_prev,
                                   uint8_t *previous_byte,
                                   uint16_t *jf_pair_count)
{
    if (*has_prev && (*previous_byte == 'J') && (byte == 'F')) {
        (*jf_pair_count)++;
    }

    *previous_byte = byte;
    *has_prev = true;
}

static void reset_xbee_diagnostics(void)
{
    g_data_router.xbee_diag_reported = false;
    g_data_router.xbee_diag_total_bytes = 0U;
    g_data_router.xbee_diag_msb_set_bytes = 0U;
    g_data_router.xbee_diag_raw_printable_bytes = 0U;
    g_data_router.xbee_diag_mask7_printable_bytes = 0U;
    g_data_router.xbee_diag_inverted_printable_bytes = 0U;
    g_data_router.xbee_diag_raw_7e_bytes = 0U;
    g_data_router.xbee_diag_mask7_7e_bytes = 0U;
    g_data_router.xbee_diag_inverted_7e_bytes = 0U;
    g_data_router.xbee_diag_raw_jf_pairs = 0U;
    g_data_router.xbee_diag_mask7_jf_pairs = 0U;
    g_data_router.xbee_diag_inverted_jf_pairs = 0U;
    g_data_router.xbee_diag_has_prev_raw = false;
    g_data_router.xbee_diag_has_prev_mask7 = false;
    g_data_router.xbee_diag_has_prev_inverted = false;
    g_data_router.xbee_diag_prev_raw = 0U;
    g_data_router.xbee_diag_prev_mask7 = 0U;
    g_data_router.xbee_diag_prev_inverted = 0U;
    g_data_router.xbee_diag_sample_len = 0U;
}

static void reset_xbee_filtered_stream_state(void)
{
    g_data_router.input_mode = INPUT_MODE_ROVER;
    g_data_router.rover_pending_j = false;
    g_data_router.rover_rx_idx = 0U;
    g_data_router.arm_packet_rx_idx = 0U;
    g_data_router.xbee_stream_mode = XBEE_STREAM_MODE_UNKNOWN;
    g_data_router.xbee_api_in_frame = false;
    g_data_router.xbee_api_escaped = false;
    g_data_router.xbee_api_length_bytes_received = 0U;
    g_data_router.xbee_api_expected_len = 0U;
    g_data_router.xbee_api_received_len = 0U;
}

static void collect_xbee_diagnostics(uint8_t raw_byte)
{
    const uint8_t mask7_byte = raw_byte & 0x7FU;
    const uint8_t inverted_byte = (uint8_t)(~raw_byte);

    g_data_router.xbee_diag_total_bytes++;

    if ((raw_byte & 0x80U) != 0U) {
        g_data_router.xbee_diag_msb_set_bytes++;
    }

    if (is_printable_stream_byte(raw_byte)) {
        g_data_router.xbee_diag_raw_printable_bytes++;
    }
    if (is_printable_stream_byte(mask7_byte)) {
        g_data_router.xbee_diag_mask7_printable_bytes++;
    }
    if (is_printable_stream_byte(inverted_byte)) {
        g_data_router.xbee_diag_inverted_printable_bytes++;
    }

    if (raw_byte == 0x7EU) {
        g_data_router.xbee_diag_raw_7e_bytes++;
    }
    if (mask7_byte == 0x7EU) {
        g_data_router.xbee_diag_mask7_7e_bytes++;
    }
    if (inverted_byte == 0x7EU) {
        g_data_router.xbee_diag_inverted_7e_bytes++;
    }

    update_jf_pair_counter(raw_byte, &g_data_router.xbee_diag_has_prev_raw,
                           &g_data_router.xbee_diag_prev_raw,
                           &g_data_router.xbee_diag_raw_jf_pairs);
    update_jf_pair_counter(mask7_byte, &g_data_router.xbee_diag_has_prev_mask7,
                           &g_data_router.xbee_diag_prev_mask7,
                           &g_data_router.xbee_diag_mask7_jf_pairs);
    update_jf_pair_counter(inverted_byte, &g_data_router.xbee_diag_has_prev_inverted,
                           &g_data_router.xbee_diag_prev_inverted,
                           &g_data_router.xbee_diag_inverted_jf_pairs);

    if (g_data_router.xbee_diag_sample_len < XBEE_DIAG_SAMPLE_MAX_LEN) {
        g_data_router.xbee_diag_sample_buf[g_data_router.xbee_diag_sample_len++] = raw_byte;
    }
}

static void log_xbee_sample_view(const char *label, uint8_t (*transform)(uint8_t))
{
    uint8_t transformed[XBEE_DIAG_SAMPLE_MAX_LEN];
    char escaped[XBEE_DIAG_SAMPLE_MAX_LEN * 4U + 1U];

    for (uint8_t i = 0U; i < g_data_router.xbee_diag_sample_len; i++) {
        transformed[i] = transform(g_data_router.xbee_diag_sample_buf[i]);
    }

    format_escaped_bytes(transformed, g_data_router.xbee_diag_sample_len,
                         escaped, sizeof(escaped));

    LOG("[downlink] xbee diag %s sample %u bytes:", label,
        g_data_router.xbee_diag_sample_len);
    for (uint8_t i = 0U; i < g_data_router.xbee_diag_sample_len; i++) {
        LOG(" %02X", transformed[i]);
    }
    LOG(" ascii=\"%s\"\r\n", escaped);
}

static uint8_t identity_transform(uint8_t byte)
{
    return byte;
}

static uint8_t mask7_transform(uint8_t byte)
{
    return byte & 0x7FU;
}

static uint8_t inverted_transform(uint8_t byte)
{
    return (uint8_t)(~byte);
}

static void maybe_log_xbee_diagnostics(void)
{
    const uint16_t total_bytes = g_data_router.xbee_diag_total_bytes;
    const uint16_t raw_printable = g_data_router.xbee_diag_raw_printable_bytes;
    const uint16_t mask7_printable = g_data_router.xbee_diag_mask7_printable_bytes;
    const uint16_t inverted_printable = g_data_router.xbee_diag_inverted_printable_bytes;
    XBeeByteDecodeMode suggested_mode = XBEE_BYTE_DECODE_MODE_RAW;
    const uint32_t msb_percent =
        (100U * (uint32_t)g_data_router.xbee_diag_msb_set_bytes) / total_bytes;
    const uint32_t raw_printable_percent =
        (100U * (uint32_t)raw_printable) / total_bytes;
    const uint32_t mask7_printable_percent =
        (100U * (uint32_t)mask7_printable) / total_bytes;
    const uint32_t inverted_printable_percent =
        (100U * (uint32_t)inverted_printable) / total_bytes;
    const char *hint = "serial format mismatch or line noise suspected";

    if (g_data_router.xbee_diag_reported ||
        (total_bytes < XBEE_DIAG_REPORT_AFTER_BYTES)) {
        return;
    }

    if ((g_data_router.xbee_diag_inverted_7e_bytes >= 4U) &&
        (g_data_router.xbee_diag_inverted_7e_bytes >
         (g_data_router.xbee_diag_raw_7e_bytes + 2U)) &&
        (inverted_printable >= raw_printable)) {
        hint = "inverted UART logic or inverting level shifter suspected";
        suggested_mode = XBEE_BYTE_DECODE_MODE_INVERTED;
    } else if ((g_data_router.xbee_diag_msb_set_bytes * 4U >= total_bytes) &&
               (mask7_printable >= (uint16_t)(raw_printable + (total_bytes / 8U)))) {
        hint = "7-bit/parity mismatch suspected";
        suggested_mode = XBEE_BYTE_DECODE_MODE_MASK7;
    } else if ((g_data_router.xbee_diag_raw_7e_bytes == 0U) &&
               (g_data_router.xbee_diag_raw_jf_pairs == 0U) &&
               (mask7_printable > raw_printable) &&
               (inverted_printable > raw_printable)) {
        hint = "received bytes do not match expected rover/arm stream";
    }

    LOG("[downlink] xbee diag summary: bytes=%u msb=%lu%% printable(raw/mask7/inv)=%lu%%/%lu%%/%lu%% 0x7E(raw/mask7/inv)=%u/%u/%u JF(raw/mask7/inv)=%u/%u/%u\r\n",
        total_bytes,
        (unsigned long)msb_percent,
        (unsigned long)raw_printable_percent,
        (unsigned long)mask7_printable_percent,
        (unsigned long)inverted_printable_percent,
        g_data_router.xbee_diag_raw_7e_bytes,
        g_data_router.xbee_diag_mask7_7e_bytes,
        g_data_router.xbee_diag_inverted_7e_bytes,
        g_data_router.xbee_diag_raw_jf_pairs,
        g_data_router.xbee_diag_mask7_jf_pairs,
        g_data_router.xbee_diag_inverted_jf_pairs);
    LOG("[downlink] xbee diag hint: %s\r\n", hint);
    log_xbee_sample_view("raw", identity_transform);
    log_xbee_sample_view("mask7", mask7_transform);
    log_xbee_sample_view("inv", inverted_transform);

    if (suggested_mode != XBEE_BYTE_DECODE_MODE_RAW) {
        g_data_router.xbee_byte_decode_mode = suggested_mode;
        reset_xbee_filtered_stream_state();
        LOG("[downlink] xbee decode mode -> %s (heuristic)\r\n",
            get_xbee_byte_decode_mode_name(suggested_mode));
    }

    g_data_router.xbee_diag_reported = true;
}

static void reset_stream_parser(void)
{
    g_data_router.xbee_byte_decode_mode = XBEE_BYTE_DECODE_MODE_RAW;
    reset_xbee_filtered_stream_state();
    reset_xbee_diagnostics();
}

static void sync_active_input_uart(bool log_change)
{
    UART_HandleTypeDef *active_uart = downlink_input_source_get_active_uart();
    UART_HandleTypeDef *previous_uart;
    bool changed = false;
    uint32_t primask = enter_critical_section();

    previous_uart = g_data_router.active_input_uart;
    if (g_data_router.active_input_uart != active_uart) {
        g_data_router.active_input_uart = active_uart;
        reset_stream_parser();
        changed = true;
    }
    exit_critical_section(primask);

    if (changed) {
        if (previous_uart != NULL) {
            stop_receive_it(previous_uart);
        }

        restart_receive_it(active_uart);
    }

    if (changed && log_change) {
        LOG("[downlink] router input -> %s\r\n",
            downlink_input_source_get_current_name());
    }
}

static void transmit_to_all_outputs(const uint8_t *data, uint16_t length)
{
    if (length == 0U) {
        return;
    }

    HAL_UART_Transmit(&ROVER_OUT_UART, (uint8_t *)data, length, HAL_MAX_DELAY);
    HAL_UART_Transmit(&ARM_OUT_UART, (uint8_t *)data, length, HAL_MAX_DELAY);
}

static void send_rover_packet(const uint8_t *packet, uint16_t length)
{
    static const uint8_t line_ending[] = "\r\n";
    char escaped[ROVER_LOG_MAX_LEN];

    if (length == 0U) {
        return;
    }

    format_escaped_bytes(packet, length, escaped, sizeof(escaped));

    if (!validate_rover_packet(packet, length)) {
        LOG("[downlink] rover rejected %u bytes:", length);
        for (uint16_t i = 0U; i < length; i++) {
            LOG(" %02X", packet[i]);
        }
        LOG(" ascii=\"%s\" (expected 0x3*/0x4* CAN text)\r\n", escaped);
        return;
    }

    LOG("[downlink] rover rx %u bytes: \"%s\"\r\n", length, escaped);
    transmit_to_all_outputs(packet, length);
    transmit_to_all_outputs(line_ending, sizeof(line_ending) - 1U);
}

static void send_arm_packet(const uint8_t *packet)
{
    LOG("[downlink] arm rx %u bytes:", ARM_PACKET_JF_SIZE);
    for (uint32_t i = 0U; i < ARM_PACKET_JF_SIZE; i++) {
        LOG(" %02X", packet[i]);
    }
    LOG("\r\n");
    transmit_to_all_outputs(packet, ARM_PACKET_JF_SIZE);
}

static void queue_arm_packet_if_ready(void)
{
    if (g_data_router.arm_packet_rx_idx < ARM_PACKET_JF_SIZE) {
        return;
    }

    if (!g_data_router.arm_packet_pending) {
        memcpy(g_data_router.arm_packet_buf, g_data_router.arm_packet_rx_buf,
               ARM_PACKET_JF_SIZE);
        g_data_router.arm_packet_pending = true;
    }

    g_data_router.arm_packet_rx_idx = 0U;
    g_data_router.input_mode = INPUT_MODE_ROVER;
}

static void queue_rover_packet_if_ready(void)
{
    if ((g_data_router.rover_rx_idx == 0U) || g_data_router.rover_packet_pending) {
        g_data_router.rover_rx_idx = 0U;
        return;
    }

    memcpy(g_data_router.rover_packet_buf, g_data_router.rover_rx_buf,
           g_data_router.rover_rx_idx);
    g_data_router.rover_packet_len = g_data_router.rover_rx_idx;
    g_data_router.rover_packet_pending = true;
    g_data_router.rover_rx_idx = 0U;
}

static void filter_input_byte(uint8_t byte)
{
    if (g_data_router.input_mode == INPUT_MODE_ARM) {
        g_data_router.arm_packet_rx_buf[g_data_router.arm_packet_rx_idx++] = byte;
        queue_arm_packet_if_ready();
        return;
    }

    if (g_data_router.rover_pending_j) {
        g_data_router.rover_pending_j = false;

        if (byte == 'F') {
            g_data_router.arm_packet_rx_buf[0] = 'J';
            g_data_router.arm_packet_rx_buf[1] = 'F';
            g_data_router.arm_packet_rx_idx = 2U;
            g_data_router.input_mode = INPUT_MODE_ARM;
            return;
        }

        if (g_data_router.rover_rx_idx < ROVER_PACKET_MAX_LEN) {
            g_data_router.rover_rx_buf[g_data_router.rover_rx_idx++] = 'J';
        } else {
            g_data_router.rover_rx_idx = 0U;
        }
    }

    if (byte == 'J') {
        g_data_router.rover_pending_j = true;
        return;
    }

    if (byte == '\n') {
        queue_rover_packet_if_ready();
        return;
    }

    if (byte == '\r') {
        return;
    }

    if (g_data_router.rover_rx_idx < ROVER_PACKET_MAX_LEN) {
        g_data_router.rover_rx_buf[g_data_router.rover_rx_idx++] = byte;
        return;
    }

    g_data_router.rover_rx_idx = 0U;
}

static void reset_xbee_api_frame_state(void)
{
    g_data_router.xbee_api_in_frame = false;
    g_data_router.xbee_api_escaped = false;
    g_data_router.xbee_api_length_bytes_received = 0U;
    g_data_router.xbee_api_expected_len = 0U;
    g_data_router.xbee_api_received_len = 0U;
}

static void start_xbee_api_frame(void)
{
    reset_xbee_api_frame_state();
    g_data_router.xbee_api_in_frame = true;
}

static void route_xbee_api_payload(const uint8_t *payload, uint16_t payload_len)
{
    for (uint16_t i = 0U; i < payload_len; i++) {
        filter_input_byte(payload[i]);
    }
}

static void handle_xbee_api_frame(void)
{
    const uint8_t *frame = g_data_router.xbee_api_frame_buf;
    const uint16_t frame_len = g_data_router.xbee_api_expected_len;
    const uint8_t frame_type = frame[0];
    const uint8_t *payload = NULL;
    uint16_t payload_len = 0U;

    if (frame_len == 0U) {
        return;
    }

    if (frame_type == 0x90U) {
        if (frame_len < 12U) {
            return;
        }
        payload = &frame[12];
        payload_len = frame_len - 12U;
    } else if (frame_type == 0x91U) {
        if (frame_len < 18U) {
            return;
        }
        payload = &frame[18];
        payload_len = frame_len - 18U;
    } else {
        return;
    }

    if (g_data_router.xbee_stream_mode != XBEE_STREAM_MODE_API) {
        g_data_router.xbee_stream_mode = XBEE_STREAM_MODE_API;
        LOG("[downlink] xbee stream detected as API mode (frame 0x%02X)\r\n",
            frame_type);
    }

    route_xbee_api_payload(payload, payload_len);
}

static void process_xbee_input_byte(uint8_t byte)
{
    uint32_t checksum_sum = 0U;

    collect_xbee_diagnostics(byte);
    maybe_log_xbee_diagnostics();
    byte = decode_xbee_stream_byte(byte);

    if (!g_data_router.xbee_api_in_frame) {
        if (byte == 0x7EU) {
            start_xbee_api_frame();
            return;
        }

        if (g_data_router.xbee_stream_mode == XBEE_STREAM_MODE_API) {
            return;
        }

        g_data_router.xbee_stream_mode = XBEE_STREAM_MODE_TRANSPARENT;
        filter_input_byte(byte);
        return;
    }

    if (g_data_router.xbee_api_escaped) {
        byte ^= 0x20U;
        g_data_router.xbee_api_escaped = false;
    } else if (byte == 0x7DU) {
        g_data_router.xbee_api_escaped = true;
        return;
    } else if (byte == 0x7EU) {
        start_xbee_api_frame();
        return;
    }

    if (g_data_router.xbee_api_length_bytes_received == 0U) {
        g_data_router.xbee_api_expected_len = ((uint16_t)byte) << 8;
        g_data_router.xbee_api_length_bytes_received = 1U;
        return;
    }

    if (g_data_router.xbee_api_length_bytes_received == 1U) {
        g_data_router.xbee_api_expected_len |= byte;
        g_data_router.xbee_api_length_bytes_received = 2U;

        if ((g_data_router.xbee_api_expected_len == 0U) ||
            (g_data_router.xbee_api_expected_len > XBEE_API_FRAME_MAX_LEN)) {
            reset_xbee_api_frame_state();
        }
        return;
    }

    if (g_data_router.xbee_api_received_len < g_data_router.xbee_api_expected_len) {
        g_data_router.xbee_api_frame_buf[g_data_router.xbee_api_received_len++] = byte;
        return;
    }

    for (uint16_t i = 0U; i < g_data_router.xbee_api_expected_len; i++) {
        checksum_sum += g_data_router.xbee_api_frame_buf[i];
    }
    checksum_sum += byte;

    if ((checksum_sum & 0xFFU) == 0xFFU) {
        handle_xbee_api_frame();
    } else if (g_data_router.xbee_stream_mode == XBEE_STREAM_MODE_UNKNOWN) {
        g_data_router.xbee_stream_mode = XBEE_STREAM_MODE_TRANSPARENT;
    }

    reset_xbee_api_frame_state();
}

static void process_selected_input_byte(UART_HandleTypeDef *huart, uint8_t byte)
{
    if (huart == &XBEE_IN_UART) {
        process_xbee_input_byte(byte);
        return;
    }

    filter_input_byte(byte);
}

void data_router_init(void)
{
    memset(&g_data_router, 0, sizeof(g_data_router));
    g_uart_trace_source = UART_TRACE_SOURCE_NONE;
    memset((void *)&g_usart6_irq_snapshot, 0, sizeof(g_usart6_irq_snapshot));
    sync_active_input_uart(true);
}

void data_router_poll(void)
{
    uint8_t arm_packet[ARM_PACKET_JF_SIZE];
    uint8_t rover_packet[ROVER_PACKET_MAX_LEN];
    uint16_t rover_packet_len = 0U;
    bool has_arm_packet = false;
    bool has_rover_packet = false;
    uint32_t primask;

    sync_active_input_uart(true);

    primask = enter_critical_section();
    if (g_data_router.arm_packet_pending) {
        memcpy(arm_packet, g_data_router.arm_packet_buf, sizeof(arm_packet));
        g_data_router.arm_packet_pending = false;
        has_arm_packet = true;
    }

    if (g_data_router.rover_packet_pending) {
        rover_packet_len = g_data_router.rover_packet_len;
        if (rover_packet_len > ROVER_PACKET_MAX_LEN) {
            rover_packet_len = ROVER_PACKET_MAX_LEN;
        }
        memcpy(rover_packet, g_data_router.rover_packet_buf, rover_packet_len);
        g_data_router.rover_packet_pending = false;
        has_rover_packet = true;
    }
    exit_critical_section(primask);

    if (has_arm_packet) {
        send_arm_packet(arm_packet);
    }

    if (has_rover_packet) {
        send_rover_packet(rover_packet, rover_packet_len);
    }
}

void data_router_on_uart_rx_complete(UART_HandleTypeDef *huart)
{
    uint8_t *rx_char = get_rx_char_slot(huart);
    uint8_t received_byte;

    if (rx_char == NULL) {
        return;
    }

    received_byte = *rx_char;
    sync_active_input_uart(false);

    if (!downlink_input_source_is_selected_uart(huart)) {
        return;
    }

    if (huart->RxState == HAL_UART_STATE_READY) {
        restart_receive_it(huart);
    }

    status_leds_on_rx_activity();
    process_selected_input_byte(huart, received_byte);
}

void data_router_on_uart_error(UART_HandleTypeDef *huart)
{
    const uint32_t pre_irq_error_flags = get_pre_irq_error_flags(huart);

    if (get_rx_char_slot(huart) == NULL) {
        return;
    }

    sync_active_input_uart(false);

    if (!downlink_input_source_is_selected_uart(huart)) {
        stop_receive_it(huart);
        return;
    }

    if (is_nonblocking_line_error(pre_irq_error_flags)) {
        return;
    }

    log_uart_error_details(huart);
    stop_receive_it(huart);
    restart_receive_it(huart);
}

void data_router_trace_usart6_irq_enter(void)
{
    g_uart_trace_source = UART_TRACE_SOURCE_USART6_IRQ;
    g_usart6_irq_snapshot.valid = true;
    g_usart6_irq_snapshot.sr = huart6.Instance->SR;
    g_usart6_irq_snapshot.cr1 = huart6.Instance->CR1;
    g_usart6_irq_snapshot.cr3 = huart6.Instance->CR3;
    g_usart6_irq_snapshot.rx_state = huart6.RxState;
    g_usart6_irq_snapshot.rx_xfer_count = huart6.RxXferCount;
}

void data_router_trace_dma2_stream1_irq_enter(void)
{
    g_uart_trace_source = UART_TRACE_SOURCE_DMA2_STREAM1_IRQ;
}

void data_router_trace_irq_exit(void)
{
    g_uart_trace_source = UART_TRACE_SOURCE_NONE;
}
