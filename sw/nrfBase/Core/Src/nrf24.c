/**
 * Martin Egli
 * 2022-03-08
 * 
 * nRF24L01-Modul
 * basierend auf msprf24, https://github.com/spirilis/msprf24
 */


// - include -------------------------------------------------------------------
#include "nrf24.h"
#include "nrf24_cmd.h"
#include "nrf24_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>

// - private app variables -----------------------------------------------------
#define cMAX_PAYLOAD_SIZE	32
static uint8_t dp2_tx_packet[cMAX_PAYLOAD_SIZE];
static uint8_t dp2_rx_packet[cMAX_PAYLOAD_SIZE];
static struct {
	uint8_t	seq;
} dp2_ctrl;

// - private variables for freeRTOS tasks --------------------------------------
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticTimer_t osStaticTimerDef_t;

/* Definitions for rf_com */
osThreadId_t rf_comHandle;
uint32_t rf_com_buffer[ 128 ];
osStaticThreadDef_t rf_com_ctrl_block;
const osThreadAttr_t rf_com_attributes = {
  .name = "rf_com",
  .cb_mem = &rf_com_ctrl_block,
  .cb_size = sizeof(rf_com_ctrl_block),
  .stack_mem = &rf_com_buffer[0],
  .stack_size = sizeof(rf_com_buffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rf_timer */
osTimerId_t rf_timerHandle;
osStaticTimerDef_t rf_timer_ctrl_block;
const osTimerAttr_t rf_timer_attributes = {
  .name = "rf_timer",
  .cb_mem = &rf_timer_ctrl_block,
  .cb_size = sizeof(rf_timer_ctrl_block),
};

void rf_com_task(void *argument);
void rf_timer_cb(void *argument);
// - private app functions -----------------------------------------------------

// - private variables ---------------------------------------------------------
volatile uint16_t nrf24_global_events;
#define nRF24_NB_JOBS	4
static struct {
	uint16_t status;    // see nRF24_STATUS_ACTIVE ...
	uint16_t nb_tries;
	uint8_t  jobs[nRF24_NB_JOBS];
	struct {
		uint8_t job;
		uint8_t result;
	} last;
	struct {
		uint8_t rxtx_addr[RF24_ADDR_SIZE];
		uint8_t	rf_ch;
		uint8_t pipe;
		uint8_t retries;
	} setup;
	struct {
		uint8_t  channel;
		uint8_t  addr[RF24_ADDR_SIZE];
		uint8_t  step;
	} scan;
	struct {
		uint16_t nb_tries;
		uint16_t guard_timing;
		uint8_t  key_code;
		uint8_t state;
	} rf_button;

} nrf24_ctrl;

#define nRF24_NB_TRIES_TO_DETECT		500	// * 6.5 ms = 3.25 s ... *10 ms  = 5.00 s

#define RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_KEYCODE_0			0
#define RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_BUTTON_KEYCODE		1

#define WAIT_FIFO_STATUS cTA_nRF24_WAIT_100us

// - jobs ----------------------------------------------------------------------
void nrf24_clear_all_jobs(void) {
	uint8_t n;
	for(n = 0; n < nRF24_NB_JOBS; n++) {
		nrf24_ctrl.jobs[n] = nRF24_JOB_NONE;
	}
}

uint8_t  nrf24_add_job(uint8_t j) {
	uint8_t n;
	for(n = 0; n < nRF24_NB_JOBS; n++) {
		if(nrf24_ctrl.jobs[n] == nRF24_JOB_NONE) {
			nrf24_ctrl.jobs[n] = j;
			return j;
		}
	}
	return nRF24_JOB_NONE;
}

#define nrf24_current_job()		nrf24_ctrl.jobs[0]	// [0] = current job
#define nrf24_peek_next_job()	nrf24_ctrl.jobs[1]	// [1] = next job

uint8_t  nrf24_get_next_job(void) {
	uint8_t n;
	for(n = 0; n < nRF24_NB_JOBS-1; n++) {
		// "shift left"
		nrf24_ctrl.jobs[n] = nrf24_ctrl.jobs[n+1];
	}
	nrf24_ctrl.jobs[n] = nRF24_JOB_NONE; // free last job
	return nrf24_ctrl.jobs[0];
}

// - public functions ----------------------------------------------------------
#define nrf24_Save_Last(j, r)	nrf24_ctrl.last.job = j; nrf24_ctrl.last.result = r;

void nrf24_Init(void) {
	nrf24_global_events = 0;
	nrf24_ctrl.status = nRF24_STATUS_OFF;
	nrf24_ctrl.setup.retries = 0x25;

	/* creation of rf_timer */
	rf_timerHandle = osTimerNew(rf_timer_cb, osTimerOnce, NULL, &rf_timer_attributes);
	/* creation of rf_com */
	rf_comHandle = osThreadNew(rf_com_task, NULL, &rf_com_attributes);

	nrf24_ctrl.rf_button.state = 0;
	nrf24_Save_Last(nRF24_JOB_NONE, nRF24_RET_PROCESSING);
	nrf24_clear_all_jobs();

	nrf24_cmd_Init();
	nrf24_hal_Init();
}

void nrf24_Load_Stored_Settings(void) {
	/*_U8 n;
	for(n = 0; n < RF24_ADDR_SIZE; n++)
		nrf24_ctrl.setup.rxtx_addr[n] = USR7_NRF24_RXTX_ADDR[n];
	nrf24_ctrl.setup.rf_ch = USR7_NRF24_RF_CH;
	nrf24_ctrl.setup.pipe = USR7_NRF24_PIPE;
	*/
//#error load default values
	return;
}

void nrf24_Open(void) {
	if(nrf24_ctrl.status & (nRF24_STATUS_ACTIVE|nRF24_STATUS_OPEN)) {
		// already open, skip
		return;
	}
	nrf24_ctrl.status |= nRF24_STATUS_ACTIVE;

	nrf24_Init();
	nrf24_Load_Stored_Settings();
	nrf24_add_job(nRF24_JOB_OPEN);
	nrf24_hal_Open();
	// wait 100 ms at least
	//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_100ms, nRF24_EV_DO_START);
}

void nrf24_Close(void) {
	//TA1_nRF24_Stop_Timeout();
	nrf24_ctrl.status = nRF24_STATUS_OFF;

	nrf24_global_events = 0;
	nrf24_clear_all_jobs();
	nrf24_hal_Close();
}

uint8_t  nrf24_Get_Current_Job(void) {
	return nrf24_current_job();
}

void nRF24_Get_Current_Address(uint8_t *addr_cpy) {
    uint8_t n;
    for(n = (RF24_ADDR_SIZE-1); n != 0; n--)
        addr_cpy[n] = nrf24_ctrl.setup.rxtx_addr[n];
    addr_cpy[n] = nrf24_ctrl.setup.rxtx_addr[n];
}
uint8_t  nRF24_Get_Current_Pipe(void) {
	return nrf24_ctrl.setup.pipe;
}
uint8_t  nRF24_Get_Current_Channel(void) {
	return nrf24_ctrl.setup.rf_ch;
}

uint8_t  nrf24_Get_Last_Job(void) {
	return nrf24_ctrl.last.job;
}

uint8_t  nrf24_Get_Last_Result(void) {
	return nrf24_ctrl.last.result;
}

uint16_t nrf24_Get_Status(void) {
    return nrf24_ctrl.status;
}

void nrf24_Set_Channel(uint8_t ch) {
	nrf24_ctrl.setup.rf_ch = ch;
}
void nrf24_Set_Pipe(uint8_t pipe) {
	nrf24_ctrl.setup.pipe = pipe;
}

uint8_t  nrf24_Set_RXTX_Addr(uint8_t *addr, uint8_t size) {
	if(size == RF24_ADDR_SIZE) {
		memcpy(nrf24_ctrl.setup.rxtx_addr, addr, size);
		return RF24_ADDR_SIZE;
	}
	return 0;
}

// - Scan for teaching address -------------------------------------------------
#define	SCAN_NB_TRIES	4	// 6: whole cycle should take 12 s, receiver is ready for 20 s 
static void nrf24_Scan_Send(void) {
	nrf24_cmd_Write_Config(0x0E);
	nrf24_cmd_Write_Setup_Retr(nrf24_ctrl.setup.retries);
	nrf24_cmd_Flush_TX();

	//nrf24_cmd_Write_TX_Payload(dp2_tx_packet, len);
	nrf24_hal_IRQ_IE_En();
	nrf24_hal_CE_Pulse();
	
	//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_10ms, nRF24_EV_GUARD_TIMEOUT);
}

static void nrf24_Scan_Set_Parameters(uint8_t ch, uint8_t a0, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4) {
	uint8_t addr[5];
	nrf24_ctrl.scan.channel = ch;
	nrf24_ctrl.scan.addr[0] = a0;
	nrf24_ctrl.scan.addr[1] = a1;
	nrf24_ctrl.scan.addr[2] = a2;
	nrf24_ctrl.scan.addr[3] = a3;
	nrf24_ctrl.scan.addr[4] = a4;
	
	nrf24_cmd_Write_RF_CH(nrf24_ctrl.scan.channel);
	memcpy(addr, nrf24_ctrl.scan.addr, RF24_ADDR_SIZE);
	nrf24_cmd_Write_RX_Addr_P0(addr);
	memcpy(addr, nrf24_ctrl.scan.addr, RF24_ADDR_SIZE);
	nrf24_cmd_Write_TX_Addr(addr);
}

static uint8_t  nrf24_Scan_Next(void) {
	if(nrf24_ctrl.nb_tries) {
		nrf24_ctrl.nb_tries--;
		return 1;// proceed
	}
	
	nrf24_ctrl.nb_tries = SCAN_NB_TRIES;
	if(nrf24_ctrl.scan.channel < 80) {
		nrf24_ctrl.scan.channel++;
		nrf24_cmd_Write_RF_CH(nrf24_ctrl.scan.channel);
		return 1;// proceed
	}
	else {
		switch(nrf24_ctrl.scan.step) {
			case 0: nrf24_ctrl.scan.step = 1; nrf24_Scan_Set_Parameters(3, 0x90, 0x69, 0x96, 0x69, 0x96); return 1;
			case 1: nrf24_ctrl.scan.step = 2; nrf24_Scan_Set_Parameters(3, 0x91, 0x69, 0x96, 0x69, 0x96); return 1;
			case 2: nrf24_ctrl.scan.step = 3; nrf24_Scan_Set_Parameters(3, 0x94, 0x69, 0x96, 0x69, 0x96); return 1;
			case 3: nrf24_ctrl.scan.step = 4; nrf24_Scan_Set_Parameters(3, 0x95, 0x69, 0x96, 0x69, 0x96); return 1;
			default: ; // done
		}
	}
	return 0; // do not proceed
}

void nrf24_Scan(void) {
	// prepare vars
	// call nrf24_Scan_Next() right after nRF24_EV_DO_START, to set channel + address correctly
	// because nrf24_Open() was not called yet, nRF does not run yet  
	nrf24_ctrl.nb_tries = 0;
	nrf24_ctrl.scan.step = 0;
	nrf24_ctrl.scan.channel = 200; // should be < 80, so all params will be set correctly @ 1st call
	// found ack using these settings: nrf24_Scan_Set_Parameters(9, 0x94, 0x69, 0x96, 0x69, 0x96);
	
	nrf24_Open();
	
	nrf24_add_job(nRF24_JOB_SCAN);
	if(nrf24_ctrl.status & nRF24_STATUS_OPEN) {
		// already open, continue with nRF24_EV_DO_START
		//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_500us, nRF24_EV_DO_START);
	}
}

// - RF Button -----------------------------------------------------------------
static void nrf24_RFButton_Send(uint8_t key) {
	uint8_t len;
	nrf24_cmd_Flush_TX();
	nrf24_cmd_Write_Config(0x0E);
	nrf24_cmd_Write_Setup_Retr(nrf24_ctrl.setup.retries);
	//len = dp2_Set_Tx_Packet_RFButton(nrf24_ctrl.rf_button.key_code[3], nrf24_ctrl.rf_button.key_code[2], nrf24_ctrl.rf_button.key_code[1], nrf24_ctrl.rf_button.key_code[0]);
	//len = dp2_Set_Tx_Packet_RFButton(0, 0, key, 0);
	nrf24_cmd_Write_TX_Payload(dp2_tx_packet, len);
	nrf24_hal_IRQ_IE_En();
	nrf24_hal_CE_Pulse();
	
	//TA1_nRF24_Wait_Event(nrf24_ctrl.rf_button.guard_timing, nRF24_EV_GUARD_TIMEOUT);
}

void nrf24_RFButton(uint8_t key, uint16_t nb_tries) {
	nrf24_Open();
	
	if((nrf24_ctrl.status & nRF24_STATUS_RECEIVER_DETECTED) == 0) {
		// 1st call, normal operation	
		nrf24_ctrl.rf_button.key_code = key;	// [1]
		nrf24_ctrl.rf_button.nb_tries = nb_tries + 8; // send additonal button released
		nrf24_ctrl.rf_button.guard_timing = 0;//cTA_nRF24_WAIT_10ms;
		
		// detect receiver first
		nrf24_ctrl.nb_tries = nRF24_NB_TRIES_TO_DETECT;
		nrf24_ctrl.rf_button.state = RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_KEYCODE_0;
	
		nrf24_add_job(nRF24_JOB_RFBUTTON);
		if(nrf24_ctrl.status & nRF24_STATUS_OPEN) {
			// already open, continue with nRF24_EV_DO_START
			//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_500us, nRF24_EV_DO_START);
		}
	}
	else {
		// add new key_code
		nrf24_ctrl.rf_button.key_code = key;	// [1]
		nrf24_ctrl.rf_button.nb_tries = nb_tries + 8; // send additonal button released

		// act as when receiver was detected and set parameters accordingly		
		// receiver detected, so set new parameters for transmission
		nrf24_ctrl.nb_tries = nrf24_ctrl.rf_button.nb_tries + 4;
		// send key_code 0 so button is released first
		nrf24_ctrl.rf_button.state = RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_KEYCODE_0;
		nrf24_ctrl.rf_button.guard_timing = 0;//cTA_nRF24_WAIT_100ms;
	}
}

// - ProcessEvents -------------------------------------------------------------
uint8_t nrf24_ProcessEvents(uint16_t events) {
	uint8_t status, value, pipe;
	uint8_t addr_copy[RF24_ADDR_SIZE];
	
	switch(nrf24_current_job()) {
		case nRF24_JOB_OPEN:
			if(events & nRF24_EV_DO_START) {
				status = nrf24_cmd_Write_Config(0x00);
				nRF24_cmd_Clear_Status();
				value = 0x73;
				status = nrf24_cmd_Transmit(0x50, &value, 1);
				
				status = nrf24_cmd_Write_Features(0x07);
				status = nrf24_cmd_Write_DYNPD(0x3F);
				// unn�tig? 
				//status = nrf24_cmd_Write_RX_PW_P4(0x20);
	
				memcpy(addr_copy, nrf24_ctrl.setup.rxtx_addr, RF24_ADDR_SIZE);	// create copy, because nrf24_cmd_Write_TX_Addr() will overwrite addr
				addr_copy[0] += nrf24_ctrl.setup.pipe;
				status = nrf24_cmd_Write_TX_Addr(addr_copy);
				memcpy(addr_copy, nrf24_ctrl.setup.rxtx_addr, RF24_ADDR_SIZE);	// create copy, because nrf24_cmd_Write_RX_Addr_Px() will overwrite addr
				addr_copy[0] += nrf24_ctrl.setup.pipe;
				status = nrf24_cmd_Write_RX_Addr_P0(addr_copy);	
				
				status = nrf24_cmd_Write_EN_AA(0x3F);
				status = nrf24_cmd_Write_EN_RXADDR(0x3F);
				status = nrf24_cmd_Write_RF_CH(nrf24_ctrl.setup.rf_ch);
				status = nrf24_cmd_Write_Setup_Retr(nrf24_ctrl.setup.retries);
				status = nrf24_cmd_Write_RF_Setup(0x07);
				status = nrf24_cmd_Write_Config(0x0E);
				status = nrf24_cmd_Flush_TX();
				
				//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_4_5ms, nRF24_EV_GUARD_TIMEOUT);
			}
			if(events & nRF24_EV_GUARD_TIMEOUT) {
				nrf24_ctrl.status |= nRF24_STATUS_OPEN;
				status = nrf24_cmd_Get_Status();
				if(nrf24_get_next_job() == nRF24_JOB_NONE) {
					// stop here, no new job, so job done
					nrf24_Save_Last(nRF24_JOB_OPEN, nRF24_RET_DONE_OK);
					nrf24_Close();
					return nRF24_RET_DONE_OK;
				}
				// start next job with nRF24_EV_DO_START
				//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_500us, nRF24_EV_DO_START);
			}
			break;
			
		// ---------------------------------------------------------------------
		
		case nRF24_JOB_SCAN:
			if(events & nRF24_EV_DO_START) {
				nrf24_Scan_Next();
				nrf24_Scan_Send();
			}
			if(events & (nRF24_EV_TX_MAX_RETRY | nRF24_EV_GUARD_TIMEOUT)) {
				nrf24_hal_CE_Low();
				nrf24_hal_IRQ_IE_Dis();
				
				if(nrf24_Scan_Next() == 0) {
					// error, could not find receiver while scanning
					nrf24_get_next_job();
					nrf24_Close();
					nrf24_Save_Last(nRF24_JOB_SCAN, nRF24_RET_ERROR);
					return nRF24_RET_ERROR;
				}				
				nrf24_Scan_Send();
			}
			if(events & nRF24_EV_RX_DONE) {
				status = nrf24_cmd_Read_RX_Payload(dp2_rx_packet, cMAX_PAYLOAD_SIZE);
				status = nrf24_cmd_Flush_RX();

				if(dp2_rx_packet[2] == 0x01) {
					// this is a ModukeID Packet, OK
					//dp2_Get_ModulID_From_Rx_Packet(dp2_rx_packet, &pipe, addr_copy);
					// also set pipe + address for communication
					// stay at this channel
					nrf24_Set_Channel(nrf24_ctrl.scan.channel);
					nrf24_Set_Pipe(pipe);
					nrf24_Set_RXTX_Addr(addr_copy, RF24_ADDR_SIZE);
					nrf24_Store_Settings_In_Flash();
					
					nrf24_get_next_job();
					nrf24_Close();
					nrf24_Save_Last(nRF24_JOB_SCAN, nRF24_RET_DONE_OK);
					return nRF24_RET_DONE_OK;
				}
			}
			break;
			
		// ---------------------------------------------------------------------
		
		case nRF24_JOB_RFBUTTON:
			if(events & nRF24_EV_DO_START) {
				nrf24_RFButton_Send(0);
			}
			if(events & (nRF24_EV_TX_MAX_RETRY | nRF24_EV_GUARD_TIMEOUT)) {
				nrf24_hal_CE_Low();
				nrf24_hal_IRQ_IE_Dis();
				

				if((nrf24_ctrl.status & nRF24_STATUS_RECEIVER_DETECTED) == 0) {
					// no receiver detected
					nrf24_ctrl.nb_tries--;
					if(nrf24_ctrl.nb_tries == 0) {
						// done, error, could not detect receiver
						
						nrf24_get_next_job();
						nrf24_Close();
						nrf24_Save_Last(nRF24_JOB_RFBUTTON, nRF24_RET_ERROR);
						return nRF24_RET_ERROR;
					}
				}
				else {
					nrf24_ctrl.nb_tries--;
					if(nrf24_ctrl.nb_tries == 0) {
						// done, OK, all packets sent
						
						nrf24_get_next_job();
						nrf24_Close();
						nrf24_Save_Last(nRF24_JOB_RFBUTTON, nRF24_RET_DONE_OK);
						return nRF24_RET_DONE_OK; 
					}
					else if(nrf24_ctrl.nb_tries == 4) {
						// send key_code 0 so button is released 
						nrf24_ctrl.rf_button.state = RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_KEYCODE_0;
					}
					else if(nrf24_ctrl.nb_tries == nrf24_ctrl.rf_button.nb_tries) {
						// send given key_code so button is pressed now
						nrf24_ctrl.rf_button.state = RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_BUTTON_KEYCODE;
					}
				}		
				if(nrf24_ctrl.rf_button.state == RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_KEYCODE_0)
					nrf24_RFButton_Send(0);
				else
					nrf24_RFButton_Send(nrf24_ctrl.rf_button.key_code);
			}
			if(events & nRF24_EV_RX_DONE) {
				status = nrf24_cmd_Read_RX_Payload(dp2_rx_packet, cMAX_PAYLOAD_SIZE);
				status = nrf24_cmd_Flush_RX();

				if((nrf24_ctrl.status & nRF24_STATUS_RECEIVER_DETECTED) == 0) {
					if(dp2_rx_packet[2] == 0x06) {
						// this is a RF_LED_ON Packet, OK
						nrf24_ctrl.status |= nRF24_STATUS_RECEIVER_DETECTED;
						// receiver detected, so set new parameters for transmission
						nrf24_ctrl.nb_tries = nrf24_ctrl.rf_button.nb_tries + 4;
						// send key_code 0 so button is released first
						nrf24_ctrl.rf_button.state = RB_BUTTON_STATE_RECEIVER_DETECTED_SEND_KEYCODE_0;
						//nrf24_ctrl.rf_button.guard_timing = cTA_nRF24_WAIT_100ms;
					}
				}
			}
			break;
			
		// ---------------------------------------------------------------------
		
		case nRF24_JOB_NONE:
		default: ;
	}
	nrf24_ctrl.last.result = nRF24_RET_PROCESSING;
	return nRF24_RET_PROCESSING;
}

void nrf24_ProcessInterrupt(void) {
	uint8_t status, value;
	nrf24_hal_IRQ_IE_Dis();
	//TA1_nRF24_Stop_Timeout();
	
	status = nRF24_cmd_Clear_Status();
	if(status & RF24_RX_DR) {
		// data in RX FIFO, maybe ACK + DATA or DATA alone
		nrf24_hal_CE_Low();
		nrf24_Send_Event(nRF24_EV_RX_DONE);
	}
	 else {
	 	if(nrf24_current_job() == nRF24_JOB_SCAN) {
			// only ACK, but no DATA
			if(status & RF24_TX_DS) {
				// do not use main_loop, becasue it may be too slow 
				// ack received, start receiving
				status = nrf24_cmd_Write_Config(0x0F);
				nrf24_hal_IRQ_IE_En();
				nrf24_hal_CE_High();
				// change guard timeout when scanning
				//TA1_nRF24_Wait_Event(cTA_nRF24_WAIT_100ms, nRF24_EV_GUARD_TIMEOUT);
			}
	 	}
	}
	if(status & RF24_MAX_RT) {
		nrf24_Send_Event(nRF24_EV_TX_MAX_RETRY);
	}
}

/* USER CODE BEGIN Header_rf_com_task_start */
/**
* @brief Function implementing the rf_com thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rf_com_task */
void rf_com_task(void *argument)
{
  /* USER CODE BEGIN rf_com_task_start */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END rf_com_task_start */
}

/* rf_timer_cb function */
void rf_timer_cb(void *argument)
{
  /* USER CODE BEGIN rf_timer_cb */

  /* USER CODE END rf_timer_cb */
}

