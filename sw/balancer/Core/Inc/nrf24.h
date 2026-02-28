/**
 * Martin Egli
 * 2022-03-08
 * 
 * nRF24L01-Modul
 * basierend auf msprf24, https://github.com/spirilis/msprf24
 */

#ifndef _NRF24_H_
#define _NRF24_H_

// - include -------------------------------------------------------------------
#include "main.h"

// - public variables ----------------------------------------------------------
extern volatile uint16_t nrf24_global_events;
#define nRF24_EV_DO_START			0x0001
#define nRF24_EV_GUARD_TIMEOUT		0x0002

#define nRF24_EV_RX_DONE			0x0010
#define nRF24_EV_TX_DONE			0x0020
#define nRF24_EV_TX_MAX_RETRY		0x0040

inline void nrf24_Send_Event(uint16_t ev) {
	//nrf24_global_events |= ev;
	//EVENTS |= fEV_WAKEUP;
}


// - public functions ----------------------------------------------------------
void nrf24_Init(void);

void nrf24_Load_Stored_Settings(void);
#define nrf24_Store_Settings_In_Flash() //void nrf24_Store_Settings_In_Flash(void);

void nrf24_Open(void);
void nrf24_Close(void);

#define nRF24_JOB_NONE			0
#define nRF24_JOB_OPEN			1
#define nRF24_JOB_SCAN			2
#define nRF24_JOB_RFBUTTON		3

uint8_t  nrf24_Get_Current_Job(void);
void nRF24_Get_Current_Address(uint8_t *cpy_addr);
uint8_t  nRF24_Get_Current_Pipe(void);
uint8_t  nRF24_Get_Current_Channel(void);
uint8_t  nrf24_Get_Last_Job(void);
uint8_t  nrf24_Get_Last_Result(void);

#define nRF24_STATUS_OFF                0x00
#define nRF24_STATUS_ACTIVE             0x01
#define nRF24_STATUS_OPEN               0x02
#define nRF24_STATUS_RECEIVER_DETECTED  0x04
uint16_t nrf24_Get_Status(void);

void nrf24_Set_Channel(uint8_t ch);
void nrf24_Set_Pipe(uint8_t pipe);
uint8_t  nrf24_Set_RXTX_Addr(uint8_t *addr, uint8_t size);

void nrf24_Scan(void);

#define nRF24_KEY_CODE_BTN1	0x10
#define nRF24_KEY_CODE_BTN2	0x20
#define nRF24_KEY_CODE_BTN3	0x40
#define nRF24_KEY_CODE_BTN4	0x80
void nrf24_RFButton(uint8_t key, uint16_t nb_tries);

#define nRF24_RET_PROCESSING	0
#define nRF24_RET_DONE_OK		1
#define nRF24_RET_ERROR			2
uint8_t nrf24_ProcessEvents(uint16_t events);

void nrf24_ProcessInterrupt(void);

#endif // _NRF24_H_
