/*
 * PL_sensor.c
 *
 *  Created on: Sep 19, 2025
 *      Author: shino
 */
#include "PL_sensor.h"

#include "adc.h"
#include "dma.h"
#include "gpio.h"//sensorLED用

#include "G_hensu.h"

uint16_t g_ADCBuffer[7];
float subV_batt[11];
//Rank順　123456
char AD_step;

uint16_t g_sensor_on[6];
uint16_t g_sensor_off[6];
uint16_t g_sensor[6][6];

/*******************************************************************/
/*	callback用関数			(pl_callback_getSensor)	*/
/*******************************************************************/
/*	DMAがスタートしたら実行するコード(AD変換複数)					*/
/*******************************************************************/
void pl_callback_getSensor(void) {
	uint16_t V_battAD;

	int j;
	HAL_ADC_Stop_DMA(&hadc1);
	switch (AD_step) {
	case 0:
		g_sensor_off[0] = g_ADCBuffer[0];
		g_sensor_off[1] = g_ADCBuffer[1];
		g_sensor_off[2] = g_ADCBuffer[2];
		g_sensor_off[3] = g_ADCBuffer[3];
		g_sensor_off[4] = g_ADCBuffer[4];
		g_sensor_off[5] = g_ADCBuffer[5];
		HAL_GPIO_WritePin(WallLED_FR_L_GPIO_Port, WallLED_FR_L_Pin, GPIO_PIN_SET);
		for (j = 0; j <= 200; j++) {
		}
		break;
	case 1:
		g_sensor_on[1] = g_ADCBuffer[1];
		g_sensor_on[3] = g_ADCBuffer[3];
		HAL_GPIO_WritePin(WallLED_FR_L_GPIO_Port, WallLED_FR_L_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WallLED_FL_R_GPIO_Port, WallLED_FL_R_Pin, GPIO_PIN_SET);
		for (j = 0; j <= 200; j++) {
		}
		break;
	case 2:
		g_sensor_on[0] = g_ADCBuffer[0];
		g_sensor_on[2] = g_ADCBuffer[2];
		HAL_GPIO_WritePin(WallLED_FL_R_GPIO_Port, WallLED_FL_R_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WallLED_NL_GPIO_Port, WallLED_NL_Pin,GPIO_PIN_SET);
		for (j = 0; j <= 200; j++) {
		}
		break;
	case 3:
		g_sensor_on[4] = g_ADCBuffer[4];
		HAL_GPIO_WritePin(WallLED_NL_GPIO_Port, WallLED_NL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WallLED_NR_GPIO_Port, WallLED_NR_Pin,GPIO_PIN_SET);
		for (j = 0; j <= 200; j++) {
		}
		break;
	case 4:
		g_sensor_on[5] = g_ADCBuffer[5];
		HAL_GPIO_WritePin(WallLED_NR_GPIO_Port, WallLED_NR_Pin,GPIO_PIN_RESET);
		for (j = 0; j <= 10; j++) {
		}
		break;
	}
	V_battAD = g_ADCBuffer[6];
	for (int i = 0; i < 9; i++) {
		subV_batt[9-i]=subV_batt[8-i];
	}
	subV_batt[0] = 3.3 * (float) V_battAD / 3979 * (100.0 + 47.0) / 47.0;
	subV_batt[10]=0.0;
	for (int i = 0; i < 10; i++) {
		subV_batt[10] += subV_batt[i];
	}
	g_V_batt=subV_batt[10]*0.1;
	AD_step++;
	if (AD_step != 5) {
		HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
	} else {
		AD_step = 0;
		for (int i = 0; i < 6; i++) {
			g_sensor[i][4] = g_sensor[i][3];
			g_sensor[i][3] = g_sensor[i][2];
			g_sensor[i][2] = g_sensor[i][1];
			g_sensor[i][1] = g_sensor[i][0];
			g_sensor[i][0] = g_sensor_on[i] - g_sensor_off[i];
			g_sensor[i][5] = (g_sensor[i][0]+g_sensor[i][1]+g_sensor[i][2]+g_sensor[i][3]+g_sensor[i][4])*0.2;
		}
	}
}

void reset_sensorbuff(){
	for (int i = 0; i < 6; i++) {
		for(int j = 0; j < 6; j++){
			g_sensor[i][j]=g_sensor[i][0];
		}
	}
}

/*******************************************************************/
/*	割り込み用動作関数(センサー取得)			(interupt_calSensor)	*/
/*******************************************************************/
/*	センサーの情報を取得する割り込み関数．						*/
/*******************************************************************/
void pl_interupt_getSensor(void) {

	HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,sizeof(g_ADCBuffer) / sizeof(uint16_t));
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ADCBuffer, sizeof(g_ADCBuffer) / sizeof(uint16_t));

}
