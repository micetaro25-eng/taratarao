/*
 * PL_gyro.h
 *
 *  Created on: Sep 29, 2025
 *      Author: shino
 */

#ifndef INC_PL_GYRO_H_
#define INC_PL_GYRO_H_

#include <stdint.h>

uint8_t read_byte( uint8_t reg );

void write_byte( uint8_t reg, uint8_t val );

void mpu6500_init( void );

float mpu6500_read_gyro_z(void);
float mpu6500_read_accel_X(void);

void mpu6500_get();
void mpu6500_do_off();
void mpu6500_offset();

extern float G_Accel_X;

#endif /* INC_PL_GYRO_H_ */
