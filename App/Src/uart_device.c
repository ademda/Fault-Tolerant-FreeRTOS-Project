/*
 * uart_device.c
 *
 *  Created on: Jan 21, 2026
 *      Author: dalya
 */

#include "uart_device.h"
#include <stdint.h>

uint8_t uart_rx_buffer[UART_FRAME_SIZE] = {0};
char *uart_tx_buffer = "received data";
