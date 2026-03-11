/**
 * Martin Egli
 * 2022-09-10
 * MPU9250 Module
 */

// - includes ------------------------------------------------------------------
#include <stdint.h>
#include <math.h>
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "mpu9250.h"
#include "main.h"
#include "spi.h"

// - defines -------------------------------------------------------------------
#define NB_ELEMENTS(x)	(sizeof(x)/sizeof(x[0]))

#define RAD_TO_DEG	(180.0f/M_PI)

// - private functions ---------------------------------------------------------

static uint16_t memcopy(uint8_t *dest, uint8_t *src, uint16_t nb_bytes) {
	uint16_t n;
	for(n = nb_bytes; n != 0; n--) {
		*dest++ = *src++;
	}
	return nb_bytes;
}

#define SENSOR_SPI SPI2
#define acc_cs_output()	// already set
#define acc_cs_set() LL_GPIO_SetOutputPin(ACC_CS_GPIO_Port, ACC_CS_Pin)
#define acc_cs_clr() LL_GPIO_ResetOutputPin(ACC_CS_GPIO_Port, ACC_CS_Pin)

static inline uint8_t spi_transfer(uint8_t b) {
	while (!LL_SPI_IsActiveFlag_TXE(SENSOR_SPI));
	LL_SPI_TransmitData8(SENSOR_SPI, b);
	while (!LL_SPI_IsActiveFlag_RXNE(SENSOR_SPI));
	return LL_SPI_ReceiveData8(SENSOR_SPI);
}

static inline uint16_t spi_transfer_buffer_blocking(uint8_t* buffer, uint16_t buf_len) {
	uint16_t n;
	if(buf_len == 0) {
		// error, nothing to send
		return 0;
	}
	acc_cs_clr();
	for(n = 0; n < buf_len; n++) {
		buffer[n] = spi_transfer(buffer[n]);
	}
	acc_cs_set();
	return n;
}

// - MPU9250 -------------------------------------------------------------------
#define READ_CMD  (0x80)
#define WRITE_CMD (0x00)

/* MPU9250
1g = 16384 (2^14)
Accelerometer Umschlagsmessung Mittelwerte
2022-09-09, Martin Egli
angle	alpha	acc x	acc y	acc z
0	90	-116.95652173913	16533.1304347826	2896.86956521739
90	0	16381.9047619048	319.619047619048	2813.42857142857
180	-90	259	-16190.7777777778	2529.44444444444
270	-180	-16359.6470588235	-9.76470588235294	5139.17647058824

Gyro, Mittelwerte in Ruhe
gyro x	gyro y	gyro z
-249.254681220314	-99.6965046888321	-266.373261985954
 */
static const float acc_1g = 16384.0f;
static const float acc_meas_x_1g[] = {16381.9047619048f, -16359.6470588235f};
static const float acc_meas_y_1g[] = {16533.1304347826f, -16190.7777777778f};
static const float gyr_meas_z_0 = -266.373261985954f;

typedef struct {
	float gain, offset;
} gain_offset_t;

static struct {
	gain_offset_t x;
	gain_offset_t y;
} mpu9250_corr_acc;
float acc_xy_angle_deg_corr = -90.0f;

static void calc_acc_xy_correction(void) {
	// calc correction values for acc_x, acc_y
	mpu9250_corr_acc.x.gain    = (2*acc_1g)/(acc_meas_x_1g[0] - acc_meas_x_1g[1]); // a = dy/dx
	mpu9250_corr_acc.x.offset = acc_1g - mpu9250_corr_acc.x.gain*acc_meas_x_1g[0]; // b = y0 - a*x0

	mpu9250_corr_acc.y.gain    = (2*acc_1g)/(acc_meas_y_1g[0] - acc_meas_y_1g[1]); // a = dy/dx
	mpu9250_corr_acc.y.offset = acc_1g - mpu9250_corr_acc.y.gain*acc_meas_y_1g[0]; // b = y0 - a*x0
}

// - public functions ----------------------------------------------------------
void mpu9250_init(void) {
	// call "MX_SPI1_Init()" for spi init
	calc_acc_xy_correction();
	mpu9250_read_who_am_i();
	mpu9250_read_config();
}


uint8_t mpu9250_read_who_am_i(void) {
  uint8_t tx_data[1+1] = {READ_CMD|117};
  spi_transfer_buffer_blocking(tx_data, NB_ELEMENTS(tx_data));
  return tx_data[1];
}

uint16_t mpu9250_read_config(void) {
  uint8_t tx_data[1+7] = {READ_CMD|26};
  return spi_transfer_buffer_blocking(tx_data, NB_ELEMENTS(tx_data));
}

uint16_t mpu9250_write_config(void) {
  uint8_t tx_data[] = {WRITE_CMD|26, 0x00, 0x00, (0x03<<2), 0x00, 0x00, 0x00};
  return spi_transfer_buffer_blocking(tx_data, NB_ELEMENTS(tx_data));
}

uint16_t mpu9250_read_int_config(void) {
  uint8_t tx_data[1+4] = {READ_CMD|55};
  return spi_transfer_buffer_blocking(tx_data, NB_ELEMENTS(tx_data));
}

uint16_t mpu9250_write_int_config(void) {
  uint8_t tx_data[] = {WRITE_CMD|55, 0xF0, 0x00, 0x00, 0x00};
  return spi_transfer_buffer_blocking(tx_data, NB_ELEMENTS(tx_data));
}

uint16_t mpu9250_read_sensor_data(uint8_t *rx_data) {
	uint8_t tx_data[MPU9250_READ_SENSOR_DATA_SIZE] = {READ_CMD|59};
	spi_transfer_buffer_blocking(tx_data, sizeof(tx_data));
	memcopy(rx_data, tx_data, sizeof(tx_data));
	return sizeof(tx_data);
}

#define MPU9250_READ_ACC_SENSOR_DATA_SIZE	(1+3*2)
uint16_t mpu9250_read_acc_data(uint8_t *rx_data) {
	uint8_t tx_data[MPU9250_READ_ACC_SENSOR_DATA_SIZE] = {READ_CMD|59};
	spi_transfer_buffer_blocking(tx_data, sizeof(tx_data));
	memcopy(rx_data, tx_data, sizeof(tx_data));
	return sizeof(tx_data);
}

#define MPU9250_READ_TEMP_SENSOR_DATA_SIZE	(1+1*2)
uint16_t mpu9250_read_temp_data(uint8_t *rx_data) {
	uint8_t tx_data[MPU9250_READ_TEMP_SENSOR_DATA_SIZE] = {READ_CMD|65};
	spi_transfer_buffer_blocking(tx_data, sizeof(tx_data));
	memcopy(rx_data, tx_data, sizeof(tx_data));
	return sizeof(tx_data);
}

#define MPU9250_READ_GYRO_SENSOR_DATA_SIZE	(1+3*2)
uint16_t mpu9250_read_gyro_data(uint8_t *rx_data) {
	uint8_t tx_data[MPU9250_READ_GYRO_SENSOR_DATA_SIZE] = {READ_CMD|67};
	spi_transfer_buffer_blocking(tx_data, sizeof(tx_data));
	memcopy(rx_data, tx_data, sizeof(tx_data));
	return sizeof(tx_data);
}

// - very application specific functions ---------------------------------------
static uint8_t mpu9250_sensor_data_buffer[MPU9250_READ_SENSOR_DATA_SIZE+8];
uint16_t mpu9250_read_sensor_values(void) {
	mpu9250_read_sensor_data(mpu9250_sensor_data_buffer);
	return 1; // OK
}

float mpu9250_get_acc_xy_angle_deg(void) {
	int16_t value;
	float acc_x, acc_y, angle_deg;
	// acc x
	value = (int16_t)((mpu9250_sensor_data_buffer[1] << 8) | mpu9250_sensor_data_buffer[2]);	// big endian
	acc_x = mpu9250_corr_acc.x.gain * (float)value + mpu9250_corr_acc.x.offset;

	// acc y
	value = (int16_t)((mpu9250_sensor_data_buffer[3] << 8) | mpu9250_sensor_data_buffer[4]);	// big endian
	acc_y = mpu9250_corr_acc.y.gain * (float)value + mpu9250_corr_acc.y.offset;

	angle_deg = (float)atan2((double)acc_x, (double)acc_y) * RAD_TO_DEG;

	return angle_deg;
}

float mpu9250_get_acc_xy_angle_correction(float angle_deg) {
	return angle_deg - 90.0f;
}

int16_t mpu9250_get_gyr_z_rate_raw(void) {
	return (int16_t)((mpu9250_sensor_data_buffer[13] << 8) | mpu9250_sensor_data_buffer[14]);
}

float mpu9250_get_gyr_z_angle_rate_deg_pro_s(void) {
	int16_t value;
	float angle_deg_pro_sec;
	// gyr z
	value = (int16_t)((mpu9250_sensor_data_buffer[13] << 8) | mpu9250_sensor_data_buffer[14]);	// big endian
	angle_deg_pro_sec = (float)value - gyr_meas_z_0;
	angle_deg_pro_sec *= (250.0f/32768.0f); // scale to +/-250 dps

	return angle_deg_pro_sec;
}
