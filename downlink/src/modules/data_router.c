#include "modules/data_router.h"

#include "debug_log.h"
#include "main.h"
#include "modules/input_source_selector.h"

#include <string.h>

#define ROVER_PACKET_MAX_LEN 64U
#define ARM_PACKET_JF_SIZE   16U

typedef enum
{
    INPUT_MODE_ROVER = 0,
    INPUT_MODE_ARM,
} InputMode;

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
} DataRouterContext;

static DataRouterContext g_data_router;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

static void start_receive_it(UART_HandleTypeDef *huart, uint8_t *rx_char)
{
    if (HAL_UART_Receive_IT(huart, rx_char, 1U) != HAL_OK) {
        Error_Handler();
    }
}

static void reset_stream_parser(void)
{
    g_data_router.input_mode = INPUT_MODE_ROVER;
    g_data_router.rover_pending_j = false;
    g_data_router.rover_rx_idx = 0U;
    g_data_router.arm_packet_rx_idx = 0U;
}

static void sync_active_input_uart(bool log_change)
{
    UART_HandleTypeDef *active_uart = downlink_input_source_get_active_uart();

    if (g_data_router.active_input_uart == active_uart) {
        return;
    }

    g_data_router.active_input_uart = active_uart;
    reset_stream_parser();

    if (log_change) {
        LOG("[downlink] router input -> %s\r\n",
            downlink_input_source_get_current_name());
    }
}

static void send_rover_packet(const uint8_t *packet, uint16_t length)
{
    static const uint8_t line_ending[] = "\r\n";

    if (length == 0U) {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)packet, length, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)line_ending, sizeof(line_ending) - 1U,
                      HAL_MAX_DELAY);
    LOG("[downlink] rover tx %u bytes\r\n", length);
}

static void send_arm_packet(const uint8_t *packet)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)packet, ARM_PACKET_JF_SIZE,
                      HAL_MAX_DELAY);
    LOG("[downlink] arm tx %u bytes\r\n", ARM_PACKET_JF_SIZE);
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

void data_router_init(void)
{
    memset(&g_data_router, 0, sizeof(g_data_router));
    sync_active_input_uart(true);

    start_receive_it(&huart3, &g_data_router.usb_rx_char);
    start_receive_it(&huart6, &g_data_router.xbee_rx_char);
}

void data_router_poll(void)
{
    uint8_t arm_packet[ARM_PACKET_JF_SIZE];
    uint8_t rover_packet[ROVER_PACKET_MAX_LEN];
    uint16_t rover_packet_len = 0U;
    bool has_arm_packet = false;
    bool has_rover_packet = false;

    sync_active_input_uart(true);

    __disable_irq();
    if (g_data_router.arm_packet_pending) {
        memcpy(arm_packet, g_data_router.arm_packet_buf, sizeof(arm_packet));
        g_data_router.arm_packet_pending = false;
        has_arm_packet = true;
    }

    if (g_data_router.rover_packet_pending) {
        rover_packet_len = g_data_router.rover_packet_len;
        memcpy(rover_packet, g_data_router.rover_packet_buf, rover_packet_len);
        g_data_router.rover_packet_pending = false;
        has_rover_packet = true;
    }
    __enable_irq();

    if (has_arm_packet) {
        send_arm_packet(arm_packet);
    }

    if (has_rover_packet) {
        send_rover_packet(rover_packet, rover_packet_len);
    }
}

void data_router_on_uart_rx_complete(UART_HandleTypeDef *huart)
{
    uint8_t received_byte;

    if (huart == &huart3) {
        received_byte = g_data_router.usb_rx_char;
        start_receive_it(&huart3, &g_data_router.usb_rx_char);
    } else if (huart == &huart6) {
        received_byte = g_data_router.xbee_rx_char;
        start_receive_it(&huart6, &g_data_router.xbee_rx_char);
    } else {
        return;
    }

    sync_active_input_uart(false);

    if (!downlink_input_source_is_selected_uart(huart)) {
        return;
    }

    filter_input_byte(received_byte);
}
