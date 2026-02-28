/**
 * Martin Egli
 * 2022-03-08
 * 
 * nRF24L01-Modul, HAL
 * basierend auf msprf24, https://github.com/spirilis/msprf24
 */

// - include -------------------------------------------------------------------
#include "nrf24_hal.h"
#include "main.h" // "io.h"
#include "spi.h"
//#include "clock.h"
//#include "timerA1.h"

// - private defines -----------------------------------------------------------
#define nRF24_SPI SPI1

// - GPIOs ---------------------------------------------------------------------

// SCK	(p5_SD_MCLK)
/*#define SCK_PIN		p5_SD_MCLK
#define sck_output()	P5DIR |=  SCK_PIN
#define sck_high()		P5OUT |=  SCK_PIN
#define sck_low()		P5OUT &= ~SCK_PIN
#define sck_sel_en()	P5SEL |=  SCK_PIN
#define sck_sel_dis()	P5SEL &= ~SCK_PIN

// MOSI
#define MOSI_PIN	p3_SD_MOSI
#define mosi_output()	P3DIR |=  MOSI_PIN
#define mosi_high()		P3OUT |=  MOSI_PIN
#define mosi_low()		P3OUT &= ~MOSI_PIN
#define mosi_sel_en()	P3SEL |=  MOSI_PIN
#define mosi_sel_dis()	P3SEL &= ~MOSI_PIN

// MISO	(p5_SD_MISO)
#define MISO_PIN	p5_SD_MISO
#define miso_output()	P5DIR |=  MISO_PIN
#define miso_input()	P5DIR &= ~MISO_PIN
#define miso_high()		P5OUT |=  MISO_PIN
#define miso_low()		P5OUT &= ~MISO_PIN
#define miso_sel_en()	P5SEL |=  MISO_PIN
#define miso_sel_dis()	P5SEL &= ~MISO_PIN
*/
// - public functions ----------------------------------------------------------
void nrf24_hal_Init(void) {
	// vars
	// gpio
	nrf24_hal_CE_Output();
	nrf24_hal_CE_Low();
	nrf24_hal_CS_Output();
	nrf24_hal_CS_Low();
	// see spi.c
	/*sck_output();
	sck_low();
	mosi_output();
	mosi_low();
	miso_input();*/
	nrf24_hal_IRQ_Input();
	nrf24_hal_IRQ_IFG_Clr();
	nrf24_hal_IRQ_IE_Dis();
	
	// see spi.c
	// spi module
	//UCx(CTL1)  =  UCSWRST;
	//SMCLK_INUSE &= ~fSMCLK_NRF24;
}

void nrf24_hal_Open(void) {
	// see spi.c
	/*
	UCx(CTL1)  = UCSSEL_3| UCSWRST;	// SMCLK, Software-Reset
    UCx(CTL0)  = UCCKPH|UCMSB|UCMST|UCSYNC;	// MSB first, Master, Synchron (SPI), UCCKPH=1, UCCKPL=0
    UCx(BRW)   = 0x01;		// SMCLK/1
    UCx(IE)    = 0;
	UCx(CTL1) &= ~UCSWRST;*/

	nrf24_hal_CE_Low();
	nrf24_hal_CS_High();
	/*sck_sel_en();
	mosi_sel_en();
	miso_sel_en();*/
	
	nrf24_hal_IRQ_IFG_Clr();
	nrf24_hal_IRQ_IE_En();
	nrf24_hal_IRQ_Input();
	nrf24_hal_IRQ_Edge_HL();

	// SPI1 konfigurieren (Beispiel)...
	LL_SPI_SetMode(nRF24_SPI, LL_SPI_MODE_MASTER);
	LL_SPI_SetDataWidth(nRF24_SPI, LL_SPI_DATAWIDTH_8BIT);
	LL_SPI_SetClockPolarity(nRF24_SPI, LL_SPI_POLARITY_LOW);
	LL_SPI_SetClockPhase(nRF24_SPI, LL_SPI_PHASE_1EDGE);
	// ... weitere Parameter

	// SPI **aktivieren** (SPE-Bit setzen)
	LL_SPI_Enable(nRF24_SPI);

}

void nrf24_hal_Close(void) {
	nrf24_hal_IRQ_IE_Dis();
	nrf24_hal_IRQ_IFG_Clr();
	nrf24_hal_IRQ_Input();
	
	// see spi.c
	//UCx(CTL1)  =  UCSWRST;
	//SMCLK_INUSE &= ~fSMCLK_NRF24;

	nrf24_hal_CE_Low();
	nrf24_hal_CS_Low();
	/*sck_sel_dis();
	sck_low();
	mosi_sel_dis();
	mosi_low();
	miso_sel_dis();
	miso_low();*/

	LL_SPI_Disable(nRF24_SPI);
}

/* Status Flags
LL_SPI_IsActiveFlag_TXE(SPI1);     // TX Buffer empty
LL_SPI_IsActiveFlag_RXNE(SPI1);    // RX Buffer not empty
LL_SPI_IsActiveFlag_BSY(SPI1);     // SPI busy
LL_SPI_IsEnabled(SPI1);            // SPI aktiv?
*/
static inline uint8_t spi_transfer(uint8_t b) {
	while (!LL_SPI_IsActiveFlag_TXE(nRF24_SPI));
	LL_SPI_TransmitData8(nRF24_SPI, b);
	while (!LL_SPI_IsActiveFlag_RXNE(nRF24_SPI));
	return LL_SPI_ReceiveData8(nRF24_SPI);
}

uint8_t  nrf24_hal_spi_Transfer(uint8_t byte) {
	return spi_transfer(byte);
}

uint8_t  nrf24_hal_spi_Transfer_U8_Ctrl_CS(uint8_t byte) {
	uint8_t rb;
	nrf24_hal_CS_Low();
	rb = spi_transfer(byte);
	nrf24_hal_CS_High();
	return rb;
}

uint16_t nrf24_hal_spi_Transfer_Buffer_Blocking(uint8_t *buffer, uint16_t buf_len) {
	uint16_t n;
	if(buf_len == 0) {
		// error, nothing to send
		return 0;
	}
	for(n = 0; n < buf_len; n++) {
		buffer[n] = spi_transfer(buffer[n]);
	}
	return n;
}

uint16_t nrf24_hal_spi_Transfer_Buffer_Blocking_Ctrl_CS(uint8_t *buffer, uint16_t buf_len) {
	uint16_t n;
	nrf24_hal_CS_Low();
	n = nrf24_hal_spi_Transfer_Buffer_Blocking(buffer, buf_len);
	nrf24_hal_CS_High();
	return n;
}
