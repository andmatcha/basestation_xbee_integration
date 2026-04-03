#include "modules/data_router.h"

#include "debug_log.h"
#include "main.h"
#include "modules/output_source_selector.h"
#include "modules/status_leds.h"

#include <stdbool.h>
#include <string.h>

#define ROVER_IN_UART                 huart1
#define ARM_IN_UART                   huart2
#define ROVER_PACKET_MAX_LEN          64U
#define ARM_PACKET_JF_SIZE            16U
#define ROUTED_PACKET_QUEUE_CAPACITY  8U

typedef enum
{
    ROUTED_PACKET_TYPE_ROVER = 0,
    ROUTED_PACKET_TYPE_ARM,
} RoutedPacketType;

typedef struct
{
    RoutedPacketType type;
    uint16_t length;
    uint8_t data[ROVER_PACKET_MAX_LEN];
} RoutedPacket;

typedef struct
{
    UART_HandleTypeDef *active_output_uart;
    uint8_t rover_rx_char;
    uint8_t arm_rx_char;
    uint8_t rover_rx_buf[ROVER_PACKET_MAX_LEN];
    uint16_t rover_rx_idx;
    bool rover_discard_until_newline;
    uint8_t arm_packet_rx_buf[ARM_PACKET_JF_SIZE];
    uint16_t arm_packet_rx_idx;
    RoutedPacket queued_packets[ROUTED_PACKET_QUEUE_CAPACITY];
    volatile uint8_t queued_packet_head;
    volatile uint8_t queued_packet_tail;
    volatile uint8_t queued_packet_count;
} DataRouterContext;

static DataRouterContext g_data_router;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

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
    if (huart == &ROVER_IN_UART) {
        return &g_data_router.rover_rx_char;
    }

    if (huart == &ARM_IN_UART) {
        return &g_data_router.arm_rx_char;
    }

    return NULL;
}

static void restart_receive_it(UART_HandleTypeDef *huart)
{
    uint8_t *rx_char = get_rx_char_slot(huart);
    HAL_StatusTypeDef status;

    if (rx_char == NULL) {
        return;
    }

    status = HAL_UART_Receive_IT(huart, rx_char, 1U);
    if (status == HAL_BUSY) {
        if (HAL_UART_AbortReceive(huart) != HAL_OK) {
            Error_Handler();
        }

        status = HAL_UART_Receive_IT(huart, rx_char, 1U);
    }

    if (status != HAL_OK) {
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

static bool enqueue_packet(RoutedPacketType type, const uint8_t *data, uint16_t length)
{
    RoutedPacket *packet;
    uint32_t primask;

    if (length > ROVER_PACKET_MAX_LEN) {
        return false;
    }

    primask = enter_critical_section();
    if (g_data_router.queued_packet_count >= ROUTED_PACKET_QUEUE_CAPACITY) {
        exit_critical_section(primask);
        return false;
    }

    packet = &g_data_router.queued_packets[g_data_router.queued_packet_tail];
    packet->type = type;
    packet->length = length;
    memcpy(packet->data, data, length);

    g_data_router.queued_packet_tail =
        (uint8_t)((g_data_router.queued_packet_tail + 1U) % ROUTED_PACKET_QUEUE_CAPACITY);
    g_data_router.queued_packet_count++;
    exit_critical_section(primask);
    return true;
}

static bool dequeue_packet(RoutedPacket *packet)
{
    uint32_t primask = enter_critical_section();

    if (g_data_router.queued_packet_count == 0U) {
        exit_critical_section(primask);
        return false;
    }

    memcpy(packet, &g_data_router.queued_packets[g_data_router.queued_packet_head],
           sizeof(*packet));
    g_data_router.queued_packet_head =
        (uint8_t)((g_data_router.queued_packet_head + 1U) % ROUTED_PACKET_QUEUE_CAPACITY);
    g_data_router.queued_packet_count--;
    exit_critical_section(primask);
    return true;
}

static void queue_rover_packet_if_ready(void)
{
    if (g_data_router.rover_rx_idx == 0U) {
        return;
    }

    if (!enqueue_packet(ROUTED_PACKET_TYPE_ROVER, g_data_router.rover_rx_buf,
                        g_data_router.rover_rx_idx)) {
        LOG("[uplink] rover queue full\r\n");
    }

    g_data_router.rover_rx_idx = 0U;
}

static void queue_arm_packet_if_ready(void)
{
    if (g_data_router.arm_packet_rx_idx < ARM_PACKET_JF_SIZE) {
        return;
    }

    if (!enqueue_packet(ROUTED_PACKET_TYPE_ARM, g_data_router.arm_packet_rx_buf,
                        ARM_PACKET_JF_SIZE)) {
        LOG("[uplink] arm queue full\r\n");
    }

    g_data_router.arm_packet_rx_idx = 0U;
}

static void process_rover_input_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        g_data_router.rover_discard_until_newline = false;
        queue_rover_packet_if_ready();
        return;
    }

    if (g_data_router.rover_discard_until_newline) {
        return;
    }

    if (g_data_router.rover_rx_idx < ROVER_PACKET_MAX_LEN) {
        g_data_router.rover_rx_buf[g_data_router.rover_rx_idx++] = byte;
        return;
    }

    g_data_router.rover_rx_idx = 0U;
    g_data_router.rover_discard_until_newline = true;
}

static void process_arm_input_byte(uint8_t byte)
{
    if (g_data_router.arm_packet_rx_idx == 0U) {
        if (byte == 'J') {
            g_data_router.arm_packet_rx_buf[0] = 'J';
            g_data_router.arm_packet_rx_idx = 1U;
        }
        return;
    }

    if (g_data_router.arm_packet_rx_idx == 1U) {
        if (byte == 'F') {
            g_data_router.arm_packet_rx_buf[1] = 'F';
            g_data_router.arm_packet_rx_idx = 2U;
            return;
        }

        g_data_router.arm_packet_rx_idx = (byte == 'J') ? 1U : 0U;
        if (byte == 'J') {
            g_data_router.arm_packet_rx_buf[0] = 'J';
        }
        return;
    }

    g_data_router.arm_packet_rx_buf[g_data_router.arm_packet_rx_idx++] = byte;
    queue_arm_packet_if_ready();
}

static void send_rover_packet(UART_HandleTypeDef *output_uart,
                              const uint8_t *packet, uint16_t length)
{
    static const uint8_t line_ending[] = "\r\n";

    if ((output_uart == NULL) || (length == 0U)) {
        return;
    }

    HAL_UART_Transmit(output_uart, (uint8_t *)packet, length, HAL_MAX_DELAY);
    HAL_UART_Transmit(output_uart, (uint8_t *)line_ending, sizeof(line_ending) - 1U,
                      HAL_MAX_DELAY);
    LOG("[uplink] rover routed %u bytes -> %s\r\n", length,
        uplink_output_source_get_current_name());
}

static void send_arm_packet(UART_HandleTypeDef *output_uart, const uint8_t *packet)
{
    if (output_uart == NULL) {
        return;
    }

    HAL_UART_Transmit(output_uart, (uint8_t *)packet, ARM_PACKET_JF_SIZE,
                      HAL_MAX_DELAY);
    LOG("[uplink] arm routed %u bytes -> %s\r\n", ARM_PACKET_JF_SIZE,
        uplink_output_source_get_current_name());
}

void data_router_init(void)
{
    memset(&g_data_router, 0, sizeof(g_data_router));
    sync_active_output_uart(true);

    restart_receive_it(&ROVER_IN_UART);
    restart_receive_it(&ARM_IN_UART);
}

void data_router_poll(void)
{
    RoutedPacket packet;

    sync_active_output_uart(true);

    while (dequeue_packet(&packet)) {
        UART_HandleTypeDef *output_uart = uplink_output_source_get_active_uart();

        if (packet.type == ROUTED_PACKET_TYPE_ARM) {
            send_arm_packet(output_uart, packet.data);
            continue;
        }

        send_rover_packet(output_uart, packet.data, packet.length);
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
    restart_receive_it(huart);
    status_leds_on_rx_activity();

    if (huart == &ROVER_IN_UART) {
        process_rover_input_byte(received_byte);
        return;
    }

    process_arm_input_byte(received_byte);
}

void data_router_on_uart_error(UART_HandleTypeDef *huart)
{
    if (get_rx_char_slot(huart) == NULL) {
        return;
    }

    LOG("[uplink] uart error on %s\r\n",
        (huart == &ROVER_IN_UART) ? "Rover IN (USART1)" : "Arm IN (USART2)");

    if (HAL_UART_AbortReceive(huart) != HAL_OK) {
        Error_Handler();
    }

    restart_receive_it(huart);
}
