/*
 * sensor.c
 *
 *  Created on: Mar 11, 2026
 *      Author: martin
 */

#include "sensor.h"
#include "mpu9250.h"

// - spi -----------------------------------------------------------------------
#define SENSOR_SPI SPI2

// - acc sensor: MPU9250 -------------------------------------------------------
#define READ_BIT (0x80)

#define ADDR_WHO_AM_I (0x75)
#define PWR_MGMT_1 (0x6B)
#define PWR_MGMT_2 (0x6C)
#define SIGNAL_PATH_RESET (0x68)

#define CONFIG (0x1A)
#define GYRO_CONFIG	(0x1B)
#define ACCEL_CONFIG (0x1C) // Messbereich einstellen
#define ACCEL_CONFIG2 (0x1D)// Low Pass Filter

#define FIFO_EN (0x23) // Daten in FIFO speichern
#define FIFO_COUNT_H (0x72) // 0x73 Anzahl Bytes
#define FIFO_R_W (0x74) // FIFO lesen

#define INT_PIN_CFG (0x37) // Interrupt Pin Setup
#define INT_ENABLE (0x38) // Interrupt aktivieren
#define INT_STATUS (0x3A) // Interrupt Status

// read values
#define ACCEL_XOUT_H (0x3B) // 0x3C	X Beschleunigung
#define ACCEL_YOUT_H (0x3D) // 0x3E	Y Beschleunigung
#define ACCEL_ZOUT_H (0x3F) // 0x40	Z Beschleunigung
#define TEMP_OUT_H (0x41) // 0x42 interne Temperatur
#define GYRO_XOUT_H	(0x43) // 0x44 Gyro X Wert
#define GYRO_YOUT_H (0x45) // 0x46 Gyro Y Wert
#define GYRO_ZOUT_H (0x47) // 0x48 Gyro Z Wert

static int16_t calc_temp(uint8_t *buffer, uint16_t buf_len, int16_t *temp) {
	if(buf_len < 14) {
		return -1;
	}
	int16_t temp_raw = (buffer[4] << 8) | buffer[5];
	// Temp (°C) = (TEMP_OUT / 333.87) + 21
	float t = temp_raw/333.87f + 21.0f;
	*temp = (int16_t)(t);
	return 0;
}

typedef struct {
	int16_t gyro[3];
	int16_t acc[3];
} gyro_acc_values_t;
#define GYRO_X (0)
#define GYRO_Y (1)
#define GYRO_Z (2)

#define ACC_X (0)
#define ACC_Y (1)
#define ACC_Z (2)

static int16_t calc_gyro_acc_values(uint8_t *buffer, uint16_t buf_len, gyro_acc_values_t *values) {
	if(buf_len < 14) {
		return -1;
	}
	values->acc[ACC_X] = (buffer[0] << 8) | buffer[1];
	values->acc[ACC_Y] = (buffer[2] << 8) | buffer[3];
	values->acc[ACC_Z] = (buffer[4] << 8) | buffer[5];

	values->gyro[GYRO_X] = (buffer[8] << 8) | buffer[9];
	values->gyro[GYRO_Y] = (buffer[10] << 8) | buffer[11];
	values->gyro[GYRO_Z] = (buffer[12] << 8) | buffer[13];
	return 0;
}

static gyro_acc_values_t my_acc_gyro;
static int16_t my_temp;
int16_t read_values(void) {
	uint8_t buf[16] = {READ_BIT|ACCEL_XOUT_H, 0};
	spi_transfer_buffer_blocking(buf, 15);
	calc_gyro_acc_values(&buf[1], 14, &my_acc_gyro);
	calc_temp(&buf[1], 14, &my_temp);
	return 0;
}














void sensor_init(void) {
	mpu9250_init();
}

void sensor_get(void) {
	uint8_t b[] = {READ_BIT|ADDR_WHO_AM_I, 0, 0, 0};
	spi_transfer_buffer_blocking(b, 4);

	/*
	write(0x6B, 0x00);   // wake up
	write(0x1B, 0x00);   // gyro ±250°/s
	write(0x1C, 0x00);   // accel ±2g
	write(0x1A, 0x03);   // low pass filter
	write(0x19, 0x04);   // sample rate
	*/
	read_values();
}



