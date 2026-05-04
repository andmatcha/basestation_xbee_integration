#include "modules/ac_packet_reducer.h"

#include <stdbool.h>

#define AC_PACKET_HEADER_LEN                 2U
#define AC_PACKET_CONTROL_MODE_BYTE_INDEX    3U
#define AC_PACKET_CONTROL_MODE_SHIFT         4U
#define AC_PACKET_CONTROL_MODE_MASK          0x03U

#define ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))

typedef enum
{
    AC_PACKET_CONTROL_MODE_IK = 0,
    AC_PACKET_CONTROL_MODE_MANUAL = 1,
    AC_PACKET_CONTROL_MODE_KEYBOARD_AUTO = 2,
} ac_packet_control_mode_t;

typedef struct
{
    uint8_t first;
    uint8_t last;
} ac_packet_byte_range_t;

typedef struct
{
    ac_packet_control_mode_t control_mode;
    uint8_t reduced_header;
    const ac_packet_byte_range_t *delete_ranges;
    size_t delete_range_count;
} ac_packet_reduction_profile_t;

static const ac_packet_byte_range_t m_packet_delete_ranges[] = {
    {3U, 3U},
    {18U, 29U},
    {31U, 36U},
};

static const ac_packet_byte_range_t i_packet_delete_ranges[] = {
    {3U, 3U},
    {8U, 13U},
    {24U, 29U},
    {31U, 36U},
};

static const ac_packet_byte_range_t b_packet_delete_ranges[] = {
    {3U, 17U},
    {24U, 29U},
    {35U, 36U},
};

static const ac_packet_reduction_profile_t reduction_profiles[] = {
    {
        AC_PACKET_CONTROL_MODE_IK,
        'I',
        i_packet_delete_ranges,
        ARRAY_LEN(i_packet_delete_ranges),
    },
    {
        AC_PACKET_CONTROL_MODE_MANUAL,
        'M',
        m_packet_delete_ranges,
        ARRAY_LEN(m_packet_delete_ranges),
    },
    {
        AC_PACKET_CONTROL_MODE_KEYBOARD_AUTO,
        'B',
        b_packet_delete_ranges,
        ARRAY_LEN(b_packet_delete_ranges),
    },
};

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

static bool is_index_in_range(uint8_t index, const ac_packet_byte_range_t *range)
{
    return (index >= range->first) && (index <= range->last);
}

static bool is_deleted_index(uint8_t index,
                             const ac_packet_reduction_profile_t *profile)
{
    for (size_t i = 0U; i < profile->delete_range_count; i++) {
        if (is_index_in_range(index, &profile->delete_ranges[i])) {
            return true;
        }
    }

    return false;
}

static const ac_packet_reduction_profile_t *find_profile(uint8_t control_mode)
{
    for (size_t i = 0U; i < ARRAY_LEN(reduction_profiles); i++) {
        if ((uint8_t)reduction_profiles[i].control_mode == control_mode) {
            return &reduction_profiles[i];
        }
    }

    return NULL;
}

static uint8_t extract_control_mode(const uint8_t *ac_packet)
{
    return (uint8_t)((ac_packet[AC_PACKET_CONTROL_MODE_BYTE_INDEX] >>
                      AC_PACKET_CONTROL_MODE_SHIFT) &
                     AC_PACKET_CONTROL_MODE_MASK);
}

static size_t calculate_reduced_len(const ac_packet_reduction_profile_t *profile)
{
    size_t reduced_len = 1U;

    for (uint8_t i = AC_PACKET_HEADER_LEN; i < AC_PACKET_REDUCER_AC_PACKET_LEN; i++) {
        if (!is_deleted_index(i, profile)) {
            reduced_len++;
        }
    }

    return reduced_len;
}

ac_packet_reducer_result_t ac_packet_reducer_reduce_for_xbee(
    const uint8_t *ac_packet,
    size_t ac_packet_len,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_len)
{
    const ac_packet_reduction_profile_t *profile;
    size_t reduced_len;
    size_t out_index = 0U;

    if (output_len != NULL) {
        *output_len = 0U;
    }

    if ((ac_packet == NULL) || (output == NULL)) {
        return AC_PACKET_REDUCER_RESULT_INVALID_ARGUMENT;
    }

    if (ac_packet_len != AC_PACKET_REDUCER_AC_PACKET_LEN) {
        return AC_PACKET_REDUCER_RESULT_INVALID_LENGTH;
    }

    if ((ac_packet[0] != 'A') || (ac_packet[1] != 'C')) {
        return AC_PACKET_REDUCER_RESULT_INVALID_HEADER;
    }

    profile = find_profile(extract_control_mode(ac_packet));
    if (profile == NULL) {
        return AC_PACKET_REDUCER_RESULT_UNSUPPORTED_MODE;
    }

    reduced_len = calculate_reduced_len(profile);
    if (output_capacity < reduced_len) {
        return AC_PACKET_REDUCER_RESULT_OUTPUT_TOO_SMALL;
    }

    output[out_index++] = profile->reduced_header;
    for (uint8_t i = AC_PACKET_HEADER_LEN; i < AC_PACKET_REDUCER_AC_PACKET_LEN; i++) {
        if (!is_deleted_index(i, profile)) {
            output[out_index++] = ac_packet[i];
        }
    }

    {
        const uint16_t crc = crc16_ccitt_false(output, out_index - sizeof(uint16_t));
        output[out_index - 2U] = (uint8_t)(crc & 0xFFU);
        output[out_index - 1U] = (uint8_t)(crc >> 8);
    }

    if (output_len != NULL) {
        *output_len = out_index;
    }

    return AC_PACKET_REDUCER_RESULT_OK;
}

const char *ac_packet_reducer_result_name(ac_packet_reducer_result_t result)
{
    switch (result) {
    case AC_PACKET_REDUCER_RESULT_OK:
        return "ok";
    case AC_PACKET_REDUCER_RESULT_INVALID_ARGUMENT:
        return "invalid argument";
    case AC_PACKET_REDUCER_RESULT_INVALID_LENGTH:
        return "invalid length";
    case AC_PACKET_REDUCER_RESULT_INVALID_HEADER:
        return "invalid header";
    case AC_PACKET_REDUCER_RESULT_UNSUPPORTED_MODE:
        return "unsupported mode";
    case AC_PACKET_REDUCER_RESULT_OUTPUT_TOO_SMALL:
        return "output too small";
    default:
        return "unknown";
    }
}
