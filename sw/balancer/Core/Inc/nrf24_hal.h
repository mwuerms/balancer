/**
 * Martin Egli
 * 2022-03-08
 * 
 * nRF24L01-Modul, HAL
 * basierend auf msprf24, https://github.com/spirilis/msprf24
 */

#ifndef _NRF24_HAL_H_
#define _NRF24_HAL_H_

// - include -------------------------------------------------------------------
#include "nrf24.h"
#include "main.h" // "io.h"

// - public gpios --------------------------------------------------------------
// CE: nRF24_CE_GPIO_Port, nRF24_CE_Pin
//#define NRF24_HAL_CE_PIN		p4_SD_PEN
#define nrf24_hal_CE_Output()	// already set
#define nrf24_hal_CE_High()		LL_GPIO_SetOutputPin(nRF_CE_GPIO_Port, nRF_CE_Pin)
#define nrf24_hal_CE_Low()		LL_GPIO_ResetOutputPin(nRF_CE_GPIO_Port, nRF_CE_Pin)
#define nrf24_hal_CE_Pulse()	nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_High(); \
								nrf24_hal_CE_Low()

// CS: nRF24_CS_GPIO_Port, nRF24_CS_Pin
//#define NRF24_HAL_CS_PIN		p4_SD_CS
#define nrf24_hal_CS_Output()	// already set
#define nrf24_hal_CS_High()		LL_GPIO_SetOutputPin(nRF_CS_GPIO_Port, nRF_CS_Pin)
#define nrf24_hal_CS_Low()		LL_GPIO_ResetOutputPin(nRF_CS_GPIO_Port, nRF_CS_Pin)

// IRQ: nRF24_IRQ_GPIO_Port, nRF24_IRQ_Pin, EXTI2_IRQn
#define nrf24_hal_IRQ_Input()	// already set
#define nrf24_hal_IRQ_IE_En()	HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0); HAL_NVIC_EnableIRQ(EXTI2_IRQn)
#define nrf24_hal_IRQ_IE_Dis()	HAL_NVIC_DisableIRQ(EXTI2_IRQn)
#define nrf24_hal_IRQ_Edge_HL()	LL_EXTI_EnableFallingTrig_0_31(EXTI2_IRQn)
#define nrf24_hal_IRQ_IFG_Clr()	LL_EXTI_ClearFlag_0_31(EXTI2_IRQn)

// - public functions ----------------------------------------------------------
void nrf24_hal_Init(void);
void nrf24_hal_Open(void);
void nrf24_hal_Close(void);

uint8_t  nrf24_hal_spi_Transfer(uint8_t b);
uint8_t  nrf24_hal_spi_Transfer_U8_Ctrl_CS(uint8_t b);
uint16_t nrf24_hal_spi_Transfer_Buffer_Blocking(uint8_t *buffer, uint16_t buf_len);
uint16_t nrf24_hal_spi_Transfer_Buffer_Blocking_Ctrl_CS(uint8_t *buffer, uint16_t buf_len);

#endif // _NRF24_HAL_H_
