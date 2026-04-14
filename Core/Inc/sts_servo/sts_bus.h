/*
 * sts_bus.h
 *
 *  Created on: Apr 11, 2026
 *      Author: miyab
 */

#ifndef INC_STS_SERVO_STS_BUS_H_
#define INC_STS_SERVO_STS_BUS_H_


#include "main.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    STS_BUS_OK = 0,
    STS_BUS_ERROR,
    STS_BUS_TIMEOUT,
    STS_BUS_BAD_RESPONSE,
    STS_BUS_BAD_CHECKSUM
} sts_bus_status_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
} sts_bus_t;

void sts_bus_init(sts_bus_t *bus,
                  UART_HandleTypeDef *huart,
                  uint32_t tx_timeout_ms,
                  uint32_t rx_timeout_ms);

sts_bus_status_t sts_bus_send(const sts_bus_t *bus,
                              const uint8_t *data,
                              uint16_t len);

sts_bus_status_t sts_bus_receive_status(const sts_bus_t *bus,
                                        uint8_t expected_id,
                                        uint8_t *rx_buf,
                                        uint16_t rx_buf_size,
                                        uint16_t *rx_len);

#endif /* INC_STS_SERVO_STS_BUS_H_ */
