/*
 * PL_gyro.c
 *
 *  Created on: Sep 29, 2025
 *      Author: shino
 */
#include "PL_gyro.h"

#include <stdio.h>
#include "spi.h"
#include "usart.h"  // printf用
#include "PL_timer.h"
#include "math.h"
#include "main.h"

#include "G_hensu.h"

#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define MPU6500_RA_CONFIG 0x00
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define ACCEL_XOUT_H 0x3D
#define ACCEL_XOUT_L 0x3E
#define GYRO_FACTOR 0x01

int Gyoff_flag;

int count_gyro;
float sumgyro_offset;
float offset;
float G_Accel_X;

/*
 @brief spi : read 1 byte
 @param uint8_t Register
 @return read 1byte data
 */
uint8_t read_byte(uint8_t reg) {
	uint8_t rx_data[2];
	uint8_t tx_data[2];

	tx_data[0] = reg | 0x80;
	tx_data[1] = 0x00;  // dummy

	HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
	HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, SET);

	return rx_data[1];
}

/*
 @brief spi : write 1 byte
 @param uint8_t Register
 @param uint8_t Write Data
 */
void write_byte(uint8_t reg, uint8_t val) {
	uint8_t rx_data[2];
	uint8_t tx_data[2];
	uint8_t data;
	tx_data[0] = reg & 0x7F;
	tx_data[1] = val;  // write data

	HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, RESET);
	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, 1);
	HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, SET);
}

/*
 * @breif initialize mpu 6500
 */

void mpu6500_init(void) {
	uint8_t who_am_i;

	HAL_Delay(100); // wait start up
	who_am_i = read_byte( WHO_AM_I); // 1. read who am i
	printf("\r\n0x%x\r\n", who_am_i); // 2. check who am i value

	// 2. error check
	if (who_am_i != 0x70) {
		while (1) {
			printf("gyro_error\r");
		}
	}

	HAL_Delay(50); // wait

	write_byte( PWR_MGMT_1, 0x00); // 3. set pwr_might

	HAL_Delay(50);

	write_byte(MPU6500_RA_CONFIG, 0x00); // 4. set config

	HAL_Delay(50);

	write_byte( GYRO_CONFIG, 0x18); // 5. set gyro config

	HAL_Delay(50);

}

/*
 * @breif read z axis
 * @return float gyro_z deg/sec
 */
float mpu6500_read_gyro_z(void) {
	int16_t gyro_z;
	float omega;

	// H:8bit shift, Link h and l
	gyro_z = (int16_t) ((int16_t) (read_byte(GYRO_ZOUT_H) << 8)
			| read_byte(GYRO_ZOUT_L));

//	omega = -((float) (gyro_z / 15.0)); // dps to deg/sec
	omega = -((float) (gyro_z / 16.4)); // dps to deg/sec

	return omega;
}

float mpu6500_read_accel_X(void) {
	int16_t accel_x;
	float omega;

	// H:8bit shift, Link h and l
	accel_x = (int16_t) ((int16_t) (read_byte(ACCEL_XOUT_H) << 8)
			| read_byte(ACCEL_XOUT_L));

	omega = -(float) (accel_x / 16.4); // dps to deg/sec

	return omega;
}


void mpu6500_get(){
	Gyro[1]=Gyro[0];
//	Gyro[0]=(mpu6500_read_gyro_z()-offset)*0.9f+Gyro[1]*0.1f;
	Gyro[0]=Gyro[1]+0.9*((mpu6500_read_gyro_z()-offset)-Gyro[1]);
//	if(Gyro[0]<1&&Gyro[0]>-1){
//		Gyro[0]=0;
//	}
	sumGyro+=Gyro[0]*0.001;
}

void mpu6500_do_off(){
	wait_ms(50);
	Gyoff_flag=1;
	Gyro[0]=0.0;
	Gyro[1]=0.0;
	sumGyro=0.0;
	wait_ms(550);
}

void mpu6500_offset(){
	if(Gyoff_flag==1){
		sumgyro_offset+=mpu6500_read_gyro_z();
		count_gyro++;
		if(count_gyro==500){
			offset=(sumgyro_offset*0.002);
			count_gyro=0;
			Gyoff_flag=0;
			sumgyro_offset=0;

			Gyro[0]=0.0;
			Gyro[1]=0.0;
			sumGyro=0.0;
		}
	}
}
