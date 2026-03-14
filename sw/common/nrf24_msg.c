/**
 * Martin Egli
 * 2026-03-01
 * 
 * nRF24L01 module for STM32
 * based on msprf24, https://github.com/spirilis/msprf24
 */

// - include -------------------------------------------------------------------
#include "nrf24.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include "usart.h"
#include "str_buf.h"

// - private variables for freeRTOS tasks --------------------------------------

osThreadId_t nrf24_msg_in_task_handle = NULL;
uint32_t nrf24_msg_in_task_buffer[ 128 ];
StaticTask_t nrf24_msg_in_task_ctrl_block;
const osThreadAttr_t nrf24_msg_in_task_attributes = {
  .name = "nrf24_msg_in_task",
  .cb_mem = &nrf24_msg_in_task_ctrl_block,
  .cb_size = sizeof(nrf24_msg_in_task_ctrl_block),
  .stack_mem = &nrf24_msg_in_task_buffer[0],
  .stack_size = sizeof(nrf24_msg_in_task_buffer),
  .priority = (osPriority_t) osPriorityLow,
};

#define NRF24_MSG_IN_QUEUE_SIZE (8)
osMessageQueueId_t nrf24_msg_in_queue_handle = NULL;
uint8_t nrf24_msg_in_queue_buffer[ NRF24_MSG_IN_QUEUE_SIZE * sizeof(nrf24_msg_t)];
StaticQueue_t nrf24_msg_in_queue_ctrl_block;
const osMessageQueueAttr_t nrf24_msg_in_queue_attributes = {
  .name = "nrf24_msg_in_queue",
  .cb_mem = &nrf24_msg_in_queue_ctrl_block,
  .cb_size = sizeof(nrf24_msg_in_queue_ctrl_block),
  .mq_mem = &nrf24_msg_in_queue_buffer,
  .mq_size = sizeof(nrf24_msg_in_queue_buffer)
};

static void nrf24_msg_in_task_cb(void *argument);

#define NRF24_OUT_STR_SIZE (256)
static char nrf24_out_str[NRF24_OUT_STR_SIZE];

void nrf24_msg_init(void) {
	if(nrf24_msg_in_task_handle == NULL) {
		nrf24_msg_in_task_handle = osThreadNew(nrf24_msg_in_task_cb, NULL, &nrf24_msg_in_task_attributes);
	}
	if(nrf24_msg_in_queue_handle == NULL) {
		nrf24_msg_in_queue_handle = osMessageQueueNew(NRF24_MSG_IN_QUEUE_SIZE, sizeof(nrf24_msg_t), &nrf24_msg_in_queue_attributes);
	}
	str_buf_clear(nrf24_out_str, NRF24_OUT_STR_SIZE);
}

void nrf24_msg_send_string(char *str) {
	nrf24_msg_t m;
	uint8_t n;

	// assume str has space in nRF24_MSG_DATA_SIZE for now, check later
	m.id = nRF24_MSG_ID_STRING;
	for(n = 0; n < nRF24_MSG_DATA_SIZE; n++) {
		m.data[n] = str[n];
		if(str[n] == 0) {
			// found end
			break;
		}
	}
	m.len = n;
	nrf24_send_message(&m);
}

void nrf24_msg_send_acc_temp_gyro_values(int16_t acc_x, int16_t acc_y, int16_t acc_z, int16_t temp, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
	nrf24_msg_t m;
	m.id = nRF24_MSG_ID_ACC_TEMP_GYRO_VALUES;
	((int16_t *)(&m.data[0]))[0] = acc_x;
	((int16_t *)(&m.data[0]))[1] = acc_y;
	((int16_t *)(&m.data[0]))[2] = acc_z;
	((int16_t *)(&m.data[0]))[3] = temp;
	((int16_t *)(&m.data[0]))[4] = gyro_x;
	((int16_t *)(&m.data[0]))[5] = gyro_y;
	((int16_t *)(&m.data[0]))[6] = gyro_z;
	m.len = 7 * sizeof(int16_t);
	nrf24_send_message(&m);
}








void nrf24_receive_packet(nrf24_msg_t *m) {
	//return
	osMessageQueuePut(nrf24_msg_in_queue_handle, m, 0, 0);
}



static void nrf24_msg_in_task_cb(void *argument) {
	nrf24_msg_t m;
	uint16_t str_len = 0;
	uint8_t new_line = '\n';

	while(1) {
		if(osMessageQueueGet(nrf24_msg_in_queue_handle, &m, 0, osWaitForever) == osOK) {
			switch(m.id) {
			// -----------------------------------------------------------------
			case nRF24_MSG_ID_ACC_TEMP_GYRO_VALUES:
				//m.len
				str_len = str_buf_clear(nrf24_out_str, NRF24_OUT_STR_SIZE);
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[0]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, ',');
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[1]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, ',');
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[2]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, ',');
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[3]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, ',');
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[4]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, ',');
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[5]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, ',');
				str_len = str_buf_append_int16(nrf24_out_str, NRF24_OUT_STR_SIZE, ((int16_t *)&m.data[0])[6]);
				str_len = str_buf_append_char(nrf24_out_str, NRF24_OUT_STR_SIZE, '\n');

				uart_send_buffer((uint8_t *)nrf24_out_str, str_len);
				break;

			// -----------------------------------------------------------------
			case nRF24_MSG_ID_STRING:
				uart_send_buffer(m.data, m.len);
				new_line = '\n';
				uart_send_buffer(&new_line, 1);
				break;

			// -----------------------------------------------------------------
			default: ;
			}

		}
	}
}
