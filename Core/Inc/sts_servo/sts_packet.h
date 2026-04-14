/*
 * sts_packet.h
 *
 *  Created on: Apr 11, 2026
 *      Author: miyab
 */

#ifndef INC_STS_SERVO_STS_PACKET_H_
#define INC_STS_SERVO_STS_PACKET_H_


#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define STS_HEADER                0xFF

#define STS_INST_PING             0x01
#define STS_INST_READ             0x02
#define STS_INST_WRITE            0x03

#define STS_BROADCAST_ID          0xFE

#define STS_MAX_PACKET_SIZE       64

uint8_t sts_checksum(const uint8_t *buf, size_t len);

/**
 * tx packet format:
 * [0]=0xFF [1]=0xFF [2]=id [3]=length [4]=inst [5...]=params [last]=checksum
 *
 * length = param_len + 2
 */
bool sts_build_packet(uint8_t id,
                      uint8_t instruction,
                      const uint8_t *params,
                      uint8_t param_len,
                      uint8_t *out_buf,
                      uint8_t *out_len);


#endif /* INC_STS_SERVO_STS_PACKET_H_ */
