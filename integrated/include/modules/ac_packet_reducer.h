#ifndef INTEGRATED_AC_PACKET_REDUCER_H
#define INTEGRATED_AC_PACKET_REDUCER_H

#include <stddef.h>
#include <stdint.h>

#define AC_PACKET_REDUCER_AC_PACKET_LEN       39U
#define AC_PACKET_REDUCER_MAX_REDUCED_LEN     19U

typedef enum
{
    AC_PACKET_REDUCER_RESULT_OK = 0,
    AC_PACKET_REDUCER_RESULT_INVALID_ARGUMENT,
    AC_PACKET_REDUCER_RESULT_INVALID_LENGTH,
    AC_PACKET_REDUCER_RESULT_INVALID_HEADER,
    AC_PACKET_REDUCER_RESULT_UNSUPPORTED_MODE,
    AC_PACKET_REDUCER_RESULT_OUTPUT_TOO_SMALL,
} ac_packet_reducer_result_t;

ac_packet_reducer_result_t ac_packet_reducer_reduce_for_xbee(
    const uint8_t *ac_packet,
    size_t ac_packet_len,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_len);

const char *ac_packet_reducer_result_name(ac_packet_reducer_result_t result);

#endif /* INTEGRATED_AC_PACKET_REDUCER_H */
