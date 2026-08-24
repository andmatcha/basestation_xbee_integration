#include "modules/ac_packet_reducer.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static unsigned int failure_count;

static void check(bool condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        failure_count++;
    }
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = (crc & 0x8000U) != 0U
                      ? (uint16_t)((crc << 1) ^ 0x1021U)
                      : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

static void test_profile(uint8_t flags,
                         uint8_t expected_header,
                         size_t expected_len,
                         const uint8_t *retained_indices,
                         size_t retained_count)
{
    uint8_t ac_packet[AC_PACKET_REDUCER_AC_PACKET_LEN];
    uint8_t output[AC_PACKET_REDUCER_MAX_REDUCED_LEN];
    size_t output_len = 0U;

    for (size_t i = 0U; i < sizeof(ac_packet); i++) {
        ac_packet[i] = (uint8_t)(i * 3U + 1U);
    }
    ac_packet[0] = 'A';
    ac_packet[1] = 'C';
    ac_packet[2] = 0xA5U;
    ac_packet[3] = flags;

    const int result = ac_packet_reducer_reduce_for_xbee(
        ac_packet, sizeof(ac_packet), output, sizeof(output), &output_len);

    check(result == AC_PACKET_REDUCER_RESULT_OK, "reduction result");
    check(output_len == expected_len, "v2 packet length");
    check(output[0] == expected_header, "packet header");
    check(output[1] == ac_packet[2], "sequence number");
    check(output[2] == flags, "flags copied without modification");
    check(retained_count + 5U == expected_len, "test field mapping length");

    for (size_t i = 0U; i < retained_count; i++) {
        check(output[3U + i] == ac_packet[retained_indices[i]],
              "retained AC field order");
    }

    const uint16_t expected_crc = crc16_ccitt_false(output, expected_len - 2U);
    const uint16_t actual_crc = (uint16_t)output[expected_len - 2U] |
                                ((uint16_t)output[expected_len - 1U] << 8);
    check(actual_crc == expected_crc, "CRC-16/CCITT-FALSE little-endian");
}

static void test_output_capacity(void)
{
    uint8_t ac_packet[AC_PACKET_REDUCER_AC_PACKET_LEN] = {'A', 'C'};
    uint8_t output[AC_PACKET_REDUCER_PACKET_M_V2_LEN - 1U];
    size_t output_len = 123U;

    ac_packet[3] = 0x11U;
    const int result = ac_packet_reducer_reduce_for_xbee(
        ac_packet, sizeof(ac_packet), output, sizeof(output), &output_len);

    check(result == AC_PACKET_REDUCER_RESULT_OUTPUT_TOO_SMALL,
          "v1-sized output buffer rejected for PacketMv2");
    check(output_len == 0U, "output length remains zero on capacity error");
}

int main(void)
{
    static const uint8_t m_retained[] = {
        4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U,
        12U, 13U, 14U, 15U, 16U, 17U, 30U,
    };
    static const uint8_t i_retained[] = {
        4U, 5U, 6U, 7U, 14U, 15U, 16U, 17U,
        18U, 19U, 20U, 21U, 22U, 23U, 30U,
    };
    static const uint8_t b_retained[] = {
        18U, 19U, 20U, 21U, 22U, 23U, 30U,
        31U, 32U, 33U, 34U,
    };

    test_profile(0x17U, 'M', AC_PACKET_REDUCER_PACKET_M_V2_LEN,
                 m_retained, sizeof(m_retained));
    test_profile(0x07U, 'I', AC_PACKET_REDUCER_PACKET_I_V2_LEN,
                 i_retained, sizeof(i_retained));
    test_profile(0x27U, 'B', AC_PACKET_REDUCER_PACKET_B_V2_LEN,
                 b_retained, sizeof(b_retained));
    test_output_capacity();

    if (failure_count != 0U) {
        fprintf(stderr, "%u test(s) failed\n", failure_count);
        return 1;
    }

    puts("AC packet reducer v2 tests passed");
    return 0;
}
