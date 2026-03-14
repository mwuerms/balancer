/**
 * Martin Egli
 * 2026-03-01
 * 
 * nRF24L01 module for STM32
 * based on msprf24, https://github.com/spirilis/msprf24
 */

#ifndef _NRF24_MSG_H_
#define _NRF24_MSG_H_

// - include -------------------------------------------------------------------
#include "main.h"

// - public definitions --------------------------------------------------------
#define nRF24_MSG_SIZE (32)
#define nRF24_MSG_DATA_SIZE (nRF24_MSG_SIZE-2)
typedef struct  __attribute__((packed)) {
	uint8_t id;
	uint8_t len;
	uint8_t data[nRF24_MSG_SIZE];
} nrf24_msg_t;

#define nRF24_MSG_ID_ACC_TEMP_GYRO_VALUES (0xA1)
//#define nRF24_MSG_ID_GYRO_VALUES (0xA2)
//#define nRF24_MSG_ID_xx_VALUES (0xA)

#define nRF24_MSG_ID_STRING (0xD0)


// - public variables ----------------------------------------------------------

// - public functions ----------------------------------------------------------

void nrf24_msg_init(void);
void nrf24_msg_send_string(char *str);
void nrf24_msg_send_acc_temp_gyro_values(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t temp, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);


void nrf24_receive_packet(nrf24_msg_t *m);

#endif // _NRF24_MSG_H_
