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

void nrf24_msg_init(void) {
	if(nrf24_msg_in_task_handle == NULL) {
		nrf24_msg_in_task_handle = osThreadNew(nrf24_msg_in_task_cb, NULL, &nrf24_msg_in_task_attributes);
	}
	if(nrf24_msg_in_queue_handle == NULL) {
		nrf24_msg_in_queue_handle = osMessageQueueNew(NRF24_MSG_IN_QUEUE_SIZE, sizeof(nrf24_msg_t), &nrf24_msg_in_queue_attributes);
	}
}

void nrf24_msg_send_string(char *str) {
	nrf24_msg_t m;
	uint8_t i, n;

	i = 0;
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

void nrf24_receive_packet(nrf24_msg_t *m) {
	//return
	osMessageQueuePut(nrf24_msg_in_queue_handle, m, 0, 0);
}

volatile static uint8_t id, len;
void nrf24_msg_parse(nrf24_msg_t *m) {
	id = m->id;
	len = m->len;
	return;
}

static void nrf24_msg_in_task_cb(void *argument) {
	nrf24_msg_t m;

	while(1) {
		if(osMessageQueueGet(nrf24_msg_in_queue_handle, &m, 0, osWaitForever) == osOK) {
			nrf24_msg_parse(&m);
		}
	}
}
