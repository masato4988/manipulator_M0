/*
 * sts_bus.c
 *
 *  Created on: Apr 11, 2026
 *      Author: miyab
 */


#include "sts_servo/sts_bus.h"
#include "sts_servo/sts_packet.h"

static bool sts_verify_status_packet(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len < 6) {
        return false;
    }

    if (buf[0] != 0xFF || buf[1] != 0xFF) {
        return false;
    }

    uint8_t expected_checksum = sts_checksum(buf, len - 1);
    return (expected_checksum == buf[len - 1]);
}

void sts_bus_init(sts_bus_t *bus,
                  UART_HandleTypeDef *huart,
                  uint32_t tx_timeout_ms,
                  uint32_t rx_timeout_ms)
{
    if (bus == NULL) {
        return;
    }

    bus->huart = huart;
    bus->tx_timeout_ms = tx_timeout_ms;
    bus->rx_timeout_ms = rx_timeout_ms;
}

sts_bus_status_t sts_bus_send(const sts_bus_t *bus,
                              const uint8_t *data,
                              uint16_t len)
{
    if (bus == NULL || bus->huart == NULL || data == NULL || len == 0) {
        return STS_BUS_ERROR;
    }

    if (HAL_UART_Transmit(bus->huart, (uint8_t *)data, len, bus->tx_timeout_ms) != HAL_OK) {
        return STS_BUS_ERROR;
    }

    return STS_BUS_OK;
}

sts_bus_status_t sts_bus_receive_status(const sts_bus_t *bus,
                                        uint8_t expected_id,
                                        uint8_t *rx_buf,
                                        uint16_t rx_buf_size,
                                        uint16_t *rx_len)
{
    if (bus == NULL || bus->huart == NULL || rx_buf == NULL || rx_len == NULL) {
        return STS_BUS_ERROR;
    }

    if (rx_buf_size < 6) {
        return STS_BUS_ERROR;
    }

    *rx_len = 0;

    // まず先頭4バイト FF FF ID LEN を読む
    if (HAL_UART_Receive(bus->huart, rx_buf, 4, bus->rx_timeout_ms) != HAL_OK) {
        return STS_BUS_TIMEOUT;
    }

    if (rx_buf[0] != 0xFF || rx_buf[1] != 0xFF) {
        return STS_BUS_BAD_RESPONSE;
    }

    if (rx_buf[2] != expected_id) {
        return STS_BUS_BAD_RESPONSE;
    }

    const uint8_t length_field = rx_buf[3];
    const uint16_t remaining = length_field; // ERROR + PARAM... + CHECKSUM

    if ((uint16_t)(4 + remaining) > rx_buf_size) {
        return STS_BUS_BAD_RESPONSE;
    }

    if (HAL_UART_Receive(bus->huart, &rx_buf[4], remaining, bus->rx_timeout_ms) != HAL_OK) {
        return STS_BUS_TIMEOUT;
    }

    *rx_len = (uint16_t)(4 + remaining);

    if (!sts_verify_status_packet(rx_buf, *rx_len)) {
        return STS_BUS_BAD_CHECKSUM;
    }

    return STS_BUS_OK;
}
