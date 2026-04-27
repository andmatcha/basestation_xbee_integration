#include "modules/data_router.h"

#include "debug_log.h"
#include "main.h"
#include "modules/input_source_selector.h"
#include "modules/status_leds.h"

#include <stdbool.h>
#include <string.h>

#define ROVER_OUT_UART               huart1
#define ARM_OUT_UART                 huart2
#define SCIENCE_OUT_UART             huart2
#define LINK_IN_UART                 huart4
#define TEXT_PACKET_MAX_LEN          128U
#define ROVER_PACKET_MAX_LEN         TEXT_PACKET_MAX_LEN
#define SCIENCE_PACKET_MAX_LEN       TEXT_PACKET_MAX_LEN
#define ARM_PACKET_JF_SIZE           16U
#define TEXT_PACKET_LOG_MAX_LEN      (TEXT_PACKET_MAX_LEN * 4U + 1U)
#define XBEE_DIAG_SAMPLE_MAX_LEN     48U
#define XBEE_DIAG_REPORT_AFTER_BYTES 128U

typedef enum
{
    INPUT_MODE_ROVER = 0,
    INPUT_MODE_ARM,
} InputMode;

typedef enum
{
    TEXT_PACKET_ROUTE_NONE = 0,
    TEXT_PACKET_ROUTE_ROVER,
    TEXT_PACKET_ROUTE_SCIENCE,
} TextPacketRoute;

typedef struct
{
    bool science_mode_enabled;
    InputMode input_mode;
    bool rover_pending_j;
    uint8_t link_rx_char;
    uint8_t rover_rx_buf[ROVER_PACKET_MAX_LEN];
    uint16_t rover_rx_idx;
    bool text_line_overflow;
    uint8_t arm_packet_rx_buf[ARM_PACKET_JF_SIZE];
    uint16_t arm_packet_rx_idx;
    volatile bool rover_packet_pending;
    volatile uint16_t rover_packet_len;
    uint8_t rover_packet_buf[ROVER_PACKET_MAX_LEN];
    volatile bool science_packet_pending;
    volatile uint16_t science_packet_len;
    uint8_t science_packet_buf[SCIENCE_PACKET_MAX_LEN];
    volatile bool arm_packet_pending;
    uint8_t arm_packet_buf[ARM_PACKET_JF_SIZE];
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

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

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
    if (huart == &LINK_IN_UART) {
        return &g_data_router.link_rx_char;
    }

    return NULL;
}

#if DEBUG_LOG_ENABLED
static const char *get_input_uart_name(const UART_HandleTypeDef *huart)
{
    return (huart == &LINK_IN_UART) ? "UART4 IN" : "unknown";
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

    LOG(" selected=%u cr1=0x%08lX cr3=0x%08lX rx_state=%lu",
        downlink_input_source_is_selected_uart(huart) ? 1U : 0U,
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

    LOG("\r\n");
}
#endif

static uint32_t get_pre_irq_error_flags(const UART_HandleTypeDef *huart)
{
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
    if (idx >= length) {
        return false;
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
    g_data_router.text_line_overflow = false;
    g_data_router.arm_packet_rx_idx = 0U;
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

#if DEBUG_LOG_ENABLED
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
#endif

static void maybe_log_xbee_diagnostics(void)
{
    const uint16_t total_bytes = g_data_router.xbee_diag_total_bytes;
    const uint16_t raw_printable = g_data_router.xbee_diag_raw_printable_bytes;
    const uint16_t mask7_printable = g_data_router.xbee_diag_mask7_printable_bytes;
    const uint16_t inverted_printable = g_data_router.xbee_diag_inverted_printable_bytes;
#if DEBUG_LOG_ENABLED
    const uint32_t msb_percent =
        (100U * (uint32_t)g_data_router.xbee_diag_msb_set_bytes) / total_bytes;
    const uint32_t raw_printable_percent =
        (100U * (uint32_t)raw_printable) / total_bytes;
    const uint32_t mask7_printable_percent =
        (100U * (uint32_t)mask7_printable) / total_bytes;
    const uint32_t inverted_printable_percent =
        (100U * (uint32_t)inverted_printable) / total_bytes;
    const char *hint = "serial format mismatch or line noise suspected";
#endif

    if (g_data_router.xbee_diag_reported ||
        (total_bytes < XBEE_DIAG_REPORT_AFTER_BYTES)) {
        return;
    }

    if ((g_data_router.xbee_diag_inverted_7e_bytes >= 4U) &&
        (g_data_router.xbee_diag_inverted_7e_bytes >
         (g_data_router.xbee_diag_raw_7e_bytes + 2U)) &&
        (inverted_printable >= raw_printable)) {
#if DEBUG_LOG_ENABLED
        hint = "inverted UART logic suspected; keeping raw bytes";
#endif
    } else if ((g_data_router.xbee_diag_msb_set_bytes * 4U >= total_bytes) &&
               (mask7_printable >= (uint16_t)(raw_printable + (total_bytes / 8U)))) {
#if DEBUG_LOG_ENABLED
        hint = "high-bit binary payload or 7-bit/parity mismatch; keeping raw bytes";
#endif
    } else if ((g_data_router.xbee_diag_raw_7e_bytes == 0U) &&
               (g_data_router.xbee_diag_raw_jf_pairs == 0U) &&
               (mask7_printable > raw_printable) &&
               (inverted_printable > raw_printable)) {
#if DEBUG_LOG_ENABLED
        hint = g_data_router.science_mode_enabled
                   ? "received bytes do not match expected rover/science stream"
                   : "received bytes do not match expected rover/arm stream";
#endif
    }

#if DEBUG_LOG_ENABLED
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
#endif

    g_data_router.xbee_diag_reported = true;
}

static void reset_stream_parser(void)
{
    reset_xbee_filtered_stream_state();
    reset_xbee_diagnostics();
}

static void sync_science_mode(void)
{
    const bool science_mode_enabled =
        downlink_input_source_is_science_mode_enabled();
    uint32_t primask;

    if (g_data_router.science_mode_enabled == science_mode_enabled) {
        return;
    }

    primask = enter_critical_section();
    g_data_router.science_mode_enabled = science_mode_enabled;
    g_data_router.rover_packet_pending = false;
    g_data_router.rover_packet_len = 0U;
    g_data_router.arm_packet_pending = false;
    g_data_router.science_packet_pending = false;
    g_data_router.science_packet_len = 0U;
    reset_stream_parser();
    exit_critical_section(primask);
}

static void send_text_packet(const char *label,
                             UART_HandleTypeDef *huart,
                             const uint8_t *packet,
                             uint16_t length)
{
    static const uint8_t line_ending[] = "\r\n";
    char escaped[TEXT_PACKET_LOG_MAX_LEN];

    if (length == 0U) {
        return;
    }

    format_escaped_bytes(packet, length, escaped, sizeof(escaped));
    LOG("[downlink] %s rx %u bytes: \"%s\"\r\n", label, length, escaped);
    HAL_UART_Transmit(huart, (uint8_t *)packet, length, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t *)line_ending, sizeof(line_ending) - 1U,
                      HAL_MAX_DELAY);
}

static void send_rover_packet(const uint8_t *packet, uint16_t length)
{
    char escaped[TEXT_PACKET_LOG_MAX_LEN];

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

    send_text_packet("rover", &ROVER_OUT_UART, packet, length);
}

static void send_arm_packet(const uint8_t *packet)
{
    LOG("[downlink] arm rx %u bytes:", ARM_PACKET_JF_SIZE);
    for (uint32_t i = 0U; i < ARM_PACKET_JF_SIZE; i++) {
        LOG(" %02X", packet[i]);
    }
    LOG("\r\n");
    HAL_UART_Transmit(&ARM_OUT_UART, (uint8_t *)packet, ARM_PACKET_JF_SIZE,
                      HAL_MAX_DELAY);
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

static TextPacketRoute classify_science_mode_text_packet(const uint8_t *packet,
                                                         uint16_t length)
{
    if (length < 7U) {
        return TEXT_PACKET_ROUTE_NONE;
    }

    if ((packet[0] != '0') || ((packet[1] != 'x') && (packet[1] != 'X'))) {
        return TEXT_PACKET_ROUTE_NONE;
    }

    if (!is_hex_digit(packet[2]) || !is_hex_digit(packet[3]) ||
        !is_hex_digit(packet[4]) || (packet[5] != ',')) {
        return TEXT_PACKET_ROUTE_NONE;
    }

    if (packet[2] == '5') {
        return TEXT_PACKET_ROUTE_SCIENCE;
    }

    if ((packet[2] == '3') || (packet[2] == '4')) {
        return TEXT_PACKET_ROUTE_ROVER;
    }

    return TEXT_PACKET_ROUTE_NONE;
}

static void queue_science_mode_text_packet_if_ready(void)
{
    const TextPacketRoute route =
        classify_science_mode_text_packet(g_data_router.rover_rx_buf,
                                          g_data_router.rover_rx_idx);

    if ((g_data_router.rover_rx_idx == 0U) || g_data_router.text_line_overflow) {
        g_data_router.rover_rx_idx = 0U;
        g_data_router.text_line_overflow = false;
        return;
    }

    if ((route == TEXT_PACKET_ROUTE_ROVER) && !g_data_router.rover_packet_pending) {
        memcpy(g_data_router.rover_packet_buf, g_data_router.rover_rx_buf,
               g_data_router.rover_rx_idx);
        g_data_router.rover_packet_len = g_data_router.rover_rx_idx;
        g_data_router.rover_packet_pending = true;
    } else if ((route == TEXT_PACKET_ROUTE_SCIENCE) &&
               !g_data_router.science_packet_pending) {
        memcpy(g_data_router.science_packet_buf, g_data_router.rover_rx_buf,
               g_data_router.rover_rx_idx);
        g_data_router.science_packet_len = g_data_router.rover_rx_idx;
        g_data_router.science_packet_pending = true;
    }

    g_data_router.rover_rx_idx = 0U;
    g_data_router.text_line_overflow = false;
}

static void filter_normal_mode_input_byte(uint8_t byte)
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

    if ((byte == 'J') && (g_data_router.rover_rx_idx == 0U)) {
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

static void filter_science_mode_input_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        queue_science_mode_text_packet_if_ready();
        return;
    }

    if (g_data_router.text_line_overflow) {
        return;
    }

    if (g_data_router.rover_rx_idx < TEXT_PACKET_MAX_LEN) {
        g_data_router.rover_rx_buf[g_data_router.rover_rx_idx++] = byte;
        return;
    }

    g_data_router.rover_rx_idx = 0U;
    g_data_router.text_line_overflow = true;
}

static void filter_input_byte(uint8_t byte)
{
    if (g_data_router.science_mode_enabled) {
        filter_science_mode_input_byte(byte);
        return;
    }

    filter_normal_mode_input_byte(byte);
}

static void process_xbee_input_byte(uint8_t byte)
{
    collect_xbee_diagnostics(byte);
    maybe_log_xbee_diagnostics();
    filter_input_byte(byte);
}

static void process_link_input_byte(uint8_t byte)
{
    process_xbee_input_byte(byte);
}

void data_router_init(void)
{
    memset(&g_data_router, 0, sizeof(g_data_router));
    g_data_router.science_mode_enabled =
        downlink_input_source_is_science_mode_enabled();
    reset_stream_parser();
    restart_receive_it(&LINK_IN_UART);
    LOG("[downlink] router input -> %s\r\n",
        downlink_input_source_get_current_name());
}

void data_router_poll(void)
{
    uint8_t arm_packet[ARM_PACKET_JF_SIZE];
    uint8_t rover_packet[ROVER_PACKET_MAX_LEN];
    uint8_t science_packet[SCIENCE_PACKET_MAX_LEN];
    uint16_t rover_packet_len = 0U;
    uint16_t science_packet_len = 0U;
    bool has_arm_packet = false;
    bool has_rover_packet = false;
    bool has_science_packet = false;
    uint32_t primask;

    sync_science_mode();

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

    if (g_data_router.science_packet_pending) {
        science_packet_len = g_data_router.science_packet_len;
        if (science_packet_len > SCIENCE_PACKET_MAX_LEN) {
            science_packet_len = SCIENCE_PACKET_MAX_LEN;
        }
        memcpy(science_packet, g_data_router.science_packet_buf, science_packet_len);
        g_data_router.science_packet_pending = false;
        has_science_packet = true;
    }
    exit_critical_section(primask);

    if (g_data_router.science_mode_enabled) {
        if (has_rover_packet) {
            send_text_packet("rover", &ROVER_OUT_UART, rover_packet, rover_packet_len);
        }

        if (has_science_packet) {
            send_text_packet("science", &SCIENCE_OUT_UART,
                             science_packet, science_packet_len);
        }
    } else {
        if (has_arm_packet) {
            send_arm_packet(arm_packet);
        }

        if (has_rover_packet) {
            send_rover_packet(rover_packet, rover_packet_len);
        }
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
    sync_science_mode();

    if (huart->RxState == HAL_UART_STATE_READY) {
        restart_receive_it(huart);
    }

    status_leds_on_rx_activity();
    process_link_input_byte(received_byte);
}

void data_router_on_uart_error(UART_HandleTypeDef *huart)
{
    const uint32_t pre_irq_error_flags = get_pre_irq_error_flags(huart);

    if (get_rx_char_slot(huart) == NULL) {
        return;
    }

    sync_science_mode();

    if (is_nonblocking_line_error(pre_irq_error_flags)) {
        return;
    }

#if DEBUG_LOG_ENABLED
    log_uart_error_details(huart);
#endif
    stop_receive_it(huart);
    restart_receive_it(huart);
}
