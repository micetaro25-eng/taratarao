/*
 * PL_ENC.c
 *
 *  Created on: Sep 21, 2025
 *      Author: shino
 */

#include "PL_ENC.h"

#include "stm32f4xx_hal.h"
#include "tim.h"
#include <math.h>

#include "motor_seigyo.h"
#include "move_preset.h"

#include "G_hensu.h"

#define Enc_CNTtoRPM 16384		//エンコーダーのカウント分解能

float WHEEL_DIAMETER_M=24.3;	//タイヤ直径mm
int count1, count2;
int enc_buff1, enc_buff2;
float EncCNT_L, EncCNT_R;
float EncRPM_L, EncRPM_R;

float RPM_pinion = 6.0 / 21.0;

void ENC_Init() {
	HAL_TIM_Base_Start_IT(&htim1); //1エンコーダーL
	HAL_TIM_Base_Start_IT(&htim8); //2エンコーダーR

	HAL_TIM_Encoder_MspInit(&htim1); //1エンコーダーL
	HAL_TIM_Encoder_MspInit(&htim8); //2エンコーダーR

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

void get_ENC() {
	enc_buff1 = TIM1->CNT;
	enc_buff2 = TIM8->CNT;
	TIM1->CNT = 0;
	TIM8->CNT = 0;
}

void math_Enc() {
	if (enc_buff1 > 32767) {
		count1 = (int16_t) enc_buff1;
	} else {
		count1 = (int16_t) enc_buff1;
	}
	if (enc_buff2 > 32767) {
		count2 = (int16_t) enc_buff2;
	} else {
		count2 = (int16_t) enc_buff2;
	}

	EncCNT_L = (float) count1;
	EncRPM_L = -EncCNT_L / Enc_CNTtoRPM;
	EncCNT_R = (float) count2;
	EncRPM_R = -EncCNT_R / Enc_CNTtoRPM;

	Enc_L[1] = Enc_L[0];
	Enc_R[1] = Enc_R[0];
	Enc_L[0] = (EncRPM_L * PI * RPM_pinion) * WHEEL_DIAMETER_M * 1000;
	Enc_R[0] = (EncRPM_R * PI * RPM_pinion) * WHEEL_DIAMETER_M * 1000;

	sumEnc_L += Enc_L[0] * 0.001;
	sumEnc_R += Enc_R[0] * 0.001;

	if (Enc_L[0] > 6000 || Enc_R[0] > 6000) {
		motor_stop_flag=1;
	}else if(fabs(Enc_L[0]-Enc_R[0])>5000){
		motor_stop_flag=1;
	}
}
