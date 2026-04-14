/*
 * sts3215.h
 *
 *  Created on: Apr 11, 2026
 *      Author: miyab
 */

#ifndef INC_STS_SERVO_STS3215_H_
#define INC_STS_SERVO_STS3215_H_

#include "sts_bus.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STS_OK = 0,
    STS_ERROR,
    STS_TIMEOUT,
    STS_PROTOCOL_ERROR
} sts_status_t;

typedef struct {
    sts_bus_t *bus;
    uint8_t id;
} sts3215_t;

void sts3215_init(sts3215_t *servo, sts_bus_t *bus, uint8_t id);

sts_status_t sts3215_ping(sts3215_t *servo);
sts_status_t sts3215_read_u8(sts3215_t *servo, uint8_t address, uint8_t *value);
sts_status_t sts3215_read_u16(sts3215_t *servo, uint8_t address, uint16_t *value);


#endif /* INC_STS_SERVO_STS3215_H_ */
