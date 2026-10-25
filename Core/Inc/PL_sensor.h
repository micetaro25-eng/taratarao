/*
 * PL_sensor.h
 *
 *  Created on: Sep 19, 2025
 *      Author: shino
 */

#ifndef INC_PL_SENSOR_H_
#define INC_PL_SENSOR_H_

#include "stm32f4xx_hal.h"

extern uint16_t g_ADCBuffer[7];
extern uint16_t g_sensor[6][6];

void pl_interupt_getSensor(void);
void pl_callback_getSensor(void);
void reset_sensorbuff();

#endif /* INC_PL_SENSOR_H_ */
