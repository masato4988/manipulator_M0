/*
 * sts_packet.c
 *
 *  Created on: Apr 11, 2026
 *      Author: miyab
 */


#include "sts_servo/sts_packet.h"

uint8_t sts_checksum(const uint8_t *buf, size_t len)
{
    uint16_t sum = 0;

    for (size_t i = 2; i < len; i++) {
        sum += buf[i];
    }

    return (uint8_t)(~(sum & 0xFF));
}

bool sts_build_packet(uint8_t id,
                      uint8_t instruction,
                      const uint8_t *params,
                      uint8_t param_len,
                      uint8_t *out_buf,
                      uint8_t *out_len)
{
    if (out_buf == NULL || out_len == NULL) {
        return false;
    }

    const uint8_t total_len = (uint8_t)(param_len + 6); // FF FF ID LEN INST PARAM... CHK

    if (total_len > STS_MAX_PACKET_SIZE) {
        return false;
    }

    out_buf[0] = STS_HEADER;
    out_buf[1] = STS_HEADER;
    out_buf[2] = id;
    out_buf[3] = (uint8_t)(param_len + 2);
    out_buf[4] = instruction;

    for (uint8_t i = 0; i < param_len; i++) {
        out_buf[5 + i] = params[i];
    }

    out_buf[5 + param_len] = sts_checksum(out_buf, 5 + param_len);
    *out_len = total_len;

    return true;
}
