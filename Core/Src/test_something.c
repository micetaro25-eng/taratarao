/*
 * test_something.c
 *
 *  Created on: Sep 19, 2025
 *      Author: shino
 */

#include "test_something.h"

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"

#include "PL_timer.h"
#include "PL_gyro.h"
#include "PL_sensor.h"
#include "motor_seigyo.h"
#include "Move_sinsura.h"
#include "Wall_Control.h"
#include "move_preset.h"
#include "PL_flash.h"
#include "Record_something.h"
#include "G_hensu.h"

void tes_gpio() {
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
}

void tes_usart1() {
	uint8_t hello[] = "Hello World¥n¥r";
	HAL_UART_Transmit(&huart2, hello, sizeof(hello), 1000);
}

void tes_usart2() {
	uint8_t hello[] = "Hello World\n\r";
	setbuf(stdout, NULL);
	printf("hello=%s", hello);
	uint8_t hoge = 3;
	printf("hoge=%d\n\r", hoge);
	while(1){
		wait_ms(1000);
		printf("PI=%f\n\r", PI);
		wait_ms(1000);
		printf("PI_=%f\n\r", PI_);
	}
}

void irumi_LED(){
	G_LED_mode=0;
	if(G_LED_mode==0){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
		wait_ms(500);
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		wait_ms(500);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		wait_ms(500);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		wait_ms(500);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		wait_ms(500);
	}
}

void check_jyro() {
	wait_ms(100);
	mpu6500_do_off();
	irumi_LED();
	while (1) {
		printf("Z=%f, sumZ=%f\n\r", Gyro[0], sumGyro);
		wait_ms(100);
	}
}

void check_jyro2() {
	wait_ms(100);
	mpu6500_do_off();
	irumi_LED();
	while (1) {
		if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
			irumi_LED();
			break;
		}
	}
	while (1) {
		printf("Z=%f, sumZ=%f\n\r",Gyro[0], sumGyro);
		wait_ms(100);
	}
}

void tes_jyro() {
	wait_ms(100);
	uint8_t gyro_config = MPU6500_ReadByte(0x1B);
	printf("GYRO_CONFIG: 0x%02X\r\n", gyro_config);  // 0x18 であるべき
	uint8_t whoami = MPU6500_ReadByte(0x75);
	printf("WHO_AM_I: 0x%02X\r\n", whoami);  // 0x70 expected
	wait_ms(100);
	mpu6500_do_off();
	while (1) {
		if (fmod(Gyro[0], 360.0f) <= 90.0f) {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
		} else if (fmod(Gyro[0], 360.0f) <= 180.0f) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		} else if (fmod(Gyro[0], 360.0) <= 270.0) {
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		}
		wait_ms(10);
	}
}

void tes_speaker() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1850); // 50% duty
	wait_ms(2000);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	wait_ms(2000);
}

void chenk_BATT() {
//	while(1){
//		printf("hello %f\n\r",g_V_batt);
//		wait_ms(1);
//	}
	float full_V=8.4;
	float botm_V=7.4;
	float rate_V;
	G_LED_mode=3;
	if(G_LED_mode==3){
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);

		while(1){
			rate_V=((g_V_batt-botm_V)/(full_V-botm_V))*100;

			if(rate_V>=80.0){
				HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
			}else if(rate_V>=60.0){
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
			}else if(rate_V>=40.0){
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
			}else if(rate_V>=20.0){
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
			}else{
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
			}
			printf("V = %f\n\r",g_V_batt);

			if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
				irumi_LED();
				break;
			}
		}
	}
}

void check_sensor() {
	while (1) {
		HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer,
				sizeof(g_ADCBuffer) / sizeof(uint16_t));
		printf("SEN1=%d,SEN2=%d,SEN3=%d,SEN4=%d,SEN5=%d,SEN6=%d\n\r",
				g_sensor[0][0], g_sensor[1][0], g_sensor[2][0], g_sensor[3][0],
				g_sensor[4][0], g_sensor[5][0]);
		wait_ms(500);
	}
}

void tes_sensor() {
	while (1) {
		if (g_sensor[0][0] >= 400) {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		} else if (g_sensor[1][0] >= 400) {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		} else if (g_sensor[2][0] >= 400) {
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		} else if (g_sensor[3][0] >= 400) {
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
		} else if (g_sensor[4][0] >= 400) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		} else if (g_sensor[5][0] >= 400) {
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		}
		wait_ms(100);
	}
}

void tes_sankaku() {
	test_san_flag=1;
	printf("start\n\r");
	while (1) {
		if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
			irumi_LED();
			Gy_En_log_flag=1;
			sakaku_daikei(0, 1000, 500, 10000, 180, 0, 0);
			sakaku_daikei(500, 2000, 500, 20000, 540, 0, 0);
			sakaku_daikei(500, 1000, 0, 10000, 180, 0, 0);
//			sakaku_daikei(0, -1000, 0, 10000, 180);
			Gy_En_log_flag=0;
			irumi_LED();
			while(1){
				printf("ready\n\r");
				while (1) {
					if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
						irumi_LED();
						break;
					}
				}
				display_En_Gy();
			}
		}
	}
}

void tes_sankaku2() {
	test_san_flag=1;
	printf("start\n\r");
	while (1) {
		if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
			irumi_LED();
			Gy_En_log_flag=1;
			sakaku_kaiten(500.0, 8000.0, 90.0, 10);
			Gy_En_log_flag=0;
			irumi_LED();
			while(1){
				printf("ready\n\r");
				while (1) {
					if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
						irumi_LED();
						break;
					}
				}
				display_En_Gy();
			}
		}
	}
}

void check_Enc() {
	while (1) {
		if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
			irumi_LED();
			break;
		}
	}
	sumEnc_L=0.0;
	sumEnc_R=0.0;
	while (1) {
		printf("%f %f\n\r", sumEnc_L, sumEnc_R);
		wait_ms(500);
	}
}

void tes_motor_Enc() {
	while (1) {
		if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
			irumi_LED();
			break;
		}
	}
	Gy_En_log_flag=1;
	// 正転方向
	HAL_GPIO_WritePin(MotorIN1_L_GPIO_Port, MotorIN1_L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MotorIN2_L_GPIO_Port, MotorIN2_L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorIN1_R_GPIO_Port, MotorIN1_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorIN2_R_GPIO_Port, MotorIN2_R_Pin, GPIO_PIN_RESET);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 20.5);	//100でfull
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 20.5);	//100でfull
	wait_ms(500);

//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);	//100でfull
//	wait_ms(500);

	// 逆転方向
	HAL_GPIO_WritePin(MotorIN1_L_GPIO_Port, MotorIN1_L_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MotorIN2_L_GPIO_Port, MotorIN2_L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MotorIN1_R_GPIO_Port, MotorIN1_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MotorIN2_R_GPIO_Port, MotorIN2_R_Pin, GPIO_PIN_SET);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 20.5);	//100でfull
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 20.5);	//100でfull
	wait_ms(500);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);	//100でfull
	Gy_En_log_flag=0;

	wait_ms(500);
	irumi_LED();
	while(1){
		printf("ready\n\r");
		while (1) {
			if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
				irumi_LED();
				break;
			}
		}
		display_En_Gy();
	}
}

void tes_tyokusin() {
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			mpu6500_do_off();

			Gy_En_log_flag=1;
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari+360.0, 4, 0);
//			kasoku(0.0, 500.0, 10000.0, 200.0, 0, 0);
//			gensoku(500.0, 0.0, 10000.0, 200.0, 0, 0);
			Gy_En_log_flag=0;

			DC_motor_stop();
			irumi_LED();
			while(1){
				printf("ready\n\r");
				while (1) {
					if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
						irumi_LED();
						break;
					}
				}
				display_En_Gy();
			}
		}
	}
}

void tes_kyuintyoku(){
	sura_tyousei_flag=0;
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			mpu6500_do_off();
			start_kyuin(20.0);
			wait_ms(1000);
			start_kyuin(25.0);
			wait_ms(300);
			start_kyuin(30.0);
			wait_ms(300);
			start_kyuin(35.0);
			wait_ms(1000);

			Gy_En_log_flag=1;
			kasoku(0.0, 1500.0, 20000.0, 180.0, 0, 0);
			gensoku(1500.0, 0.0, 20000.0, 180.0, 0, 0);
			Gy_En_log_flag=0;

			DC_motor_stop();
			stop_kyuin();
			irumi_LED();
			while(1){
				printf("ready\n\r");
				while (1) {
					if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
						irumi_LED();
						break;
					}
				}
				display_En_Gy();
			}
		}
	}
}

void tes_kaiten() {
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			mpu6500_do_off();

			Gy_En_log_flag=1;
			sakaku_kaiten(500.0, 8000.0, 90.0, 10);
			Gy_En_log_flag=0;

			DC_motor_stop();
			irumi_LED();
			while(1){
				printf("ready\n\r");
				while (1) {
					if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
						irumi_LED();
						break;
					}
				}
				display_En_Gy();
			}
		}
	}
}

void tes_judgewall(){
	int Lkabe1, Lkabe2;
	int Rkabe1, Rkabe2;
	int FLkabe1, FLkabe2;
	int FRkabe1, FRkabe2;
	while(1){
		if(g_sensor[2][0]>300){
			Lkabe1=g_sensor[1][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[2][0]>300){
			Lkabe2=g_sensor[1][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[1][0]>300){
			Rkabe1=g_sensor[2][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[1][0]>300){
			Rkabe2=g_sensor[2][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[2][0]>300){
			FLkabe1=g_sensor[0][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[1][0]>300){
			FRkabe1=g_sensor[3][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[2][0]>300){
			FLkabe2=g_sensor[0][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if(g_sensor[1][0]>300){
			FRkabe2=g_sensor[3][0];
			irumi_LED();
			break;
		}
	}
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			printf("L1=%d L2=%d R1=%d R2=%d F1=%d F2=%d\n\r",Lkabe1,Lkabe2,Rkabe1,Rkabe2,(FLkabe1+FRkabe1),(FLkabe2+FRkabe2));
		}
	}
}

void tes_sendiff(){
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			break;
		}
	}
	mpu6500_do_off();
	sensor_log_flag = 1;
	sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari+20, 4, 0);
	tohasi_log_flag =1;
	SURAKABE_F=g_sensor[0][0]+g_sensor[3][0];
	wait_ms(200);
	DC_motor_stop();
	sensor_log_flag = 0;
	while(1){
		printf("ready\n\r");
		while (1) {
			if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
				irumi_LED();
				break;
			}
		}
		display_sensor_log();
		printf("tohasira %f\n\r",tohasi_log-20.0);
	}
}
void cary_fkou(){
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			break;
		}
	}
	int i_carry_fkou,i_step;
	float t_carry;
	if(G_kukaku==180){
		double V_diff_sum;
		i_carry_fkou=0;
		g_Pe_KbSt=0.0;
		g_Ie_KbSt=0.0;
		g_De_KbSt=0.0;
//		G_subPj_Str=0.0;
//		G_subIj_Str=0.0;
//		G_subDj_Str=0.0;
//		G_subFA_Str=0.0;
//		G_subFV_Str=0.0;
		kasoku(0.0, 200.0, 10000.0, 180.0, 0, 0);
		Gy_En_log_flag=1;
		G_timer=0;
		while(1){
			if(i_carry_fkou<10){
				V_diff_sum=0;
				i_step=0;
				V_diff_sum+=(G_V_Motor-(sumEnc_L+sumEnc_R)*0.5)*0.001;
				t_carry=G_timer;
				i_step++;
				while(1){
					if(i_step<10){
						if(t_carry+1==G_timer){
							V_diff_sum+=(G_V_Motor-((sumEnc_L+sumEnc_R)*0.5))*0.001;
							t_carry=G_timer;
							i_step++;
						}
					}else{
						break;
					}
				}
//				printf("%f\n\r",V_diff_sum);
				if(V_diff_sum<1.0){
					break;
				}else{
					i_carry_fkou++;
					g_FV_KbSt+=V_diff_sum*0.001;
					wait_ms(50);
				}
			}else{
				break;
			}
		}
		Gy_En_log_flag=0;
		gensoku(500.0, 0.0, 8000.0, 180.0, 0, 0);
		while(1){
			if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
				irumi_LED();
				printf("V_diff_sum=%f\n\r", V_diff_sum);
				printf("G_subFV_Str=%f\n\r", g_FV_KbSt);
				display_En_Gy();
			}
		}
	}
}
void cary_sen(){
	int sumCFL, sumCFR, sumCL, sumCR;
	int sumNoCFL, sumNoCFR, sumNoCL, sumNoCR;
	int sumSURFL, sumSURFR, sumNFL, sumNFR;
	int cnt_sencary, cntMAX_sencary=5;
	float hasira1,hasira2, hasira3;
	sumCFL=0.0;
	sumCFR=0.0;
	sumCL=0.0;
	sumCR=0.0;
	sumNoCFL=0.0;
	sumNoCFR=0.0;
	sumNoCL=0.0;
	sumNoCR=0.0;
	sumSURFL=0.0;
	sumSURFR=0.0;
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			break;
		}
	}
	mpu6500_do_off();
	for(cnt_sencary=0; cnt_sencary<cntMAX_sencary;cnt_sencary++){
		sumCL+=g_sensor[1][0];
		sumCR+=g_sensor[2][0];
		sumNoCFL+=g_sensor[0][0];
		sumNoCFR+=g_sensor[3][0];
		wait_ms(50);
	}
	CENTER_L=sumCL/cntMAX_sencary;
	CENTER_R=sumCR/cntMAX_sencary;
	kasoku(0.0, 500.0, 10000.0, hazimari-(G_tansakumae*2), 4, 0);
	gensoku(500.0, 0.0, 10000.0, (G_tansakumae*2), 0, 0);
	tohasi_log_flag =1;
	wait_ms(200);
	hasira1=tohasi_log;
	DC_motor_stop();

	for(cnt_sencary=0; cnt_sencary<cntMAX_sencary;cnt_sencary++){
		sumNFL+=g_sensor[0][0];
		sumNFR+=g_sensor[3][0];
		wait_ms(50);
	}
	THRESHOLD_naFL=(int)((sumSURFL/cntMAX_sencary)*0.7);
	THRESHOLD_naFR=(int)((sumSURFR/cntMAX_sencary)*0.7);

	irumi_LED();
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			break;
		}
	}
	tohasi_log=0;
	mpu6500_do_off();
	kasoku(0.0, 500.0, 10000.0, hazimari-(G_tansakumae*2), 4, 0);
	gensoku(500.0, 0.0, 10000.0, (G_tansakumae*3), 0, 0);
	wait_ms(200);
	hasira2=tohasi_log-20.0;
	DC_motor_stop();
	for(cnt_sencary=0; cnt_sencary<cntMAX_sencary;cnt_sencary++){
		sumSURFL+=g_sensor[0][0];
		sumSURFR+=g_sensor[3][0];
		sumNoCL+=g_sensor[1][0];
		sumNoCR+=g_sensor[2][0];
		wait_ms(50);
	}
	SURAKABE_F=(sumSURFL+sumSURFR)/cntMAX_sencary;
	THRESHOLD_FL=(int)((sumNoCFL/cntMAX_sencary)+(((sumSURFL/cntMAX_sencary)-(sumNoCFL/cntMAX_sencary))*0.25));
	THRESHOLD_FR=(int)((sumNoCFR/cntMAX_sencary)+(((sumSURFR/cntMAX_sencary)-(sumNoCFR/cntMAX_sencary))*0.25));
	THRESHOLD_L=(int)((sumNoCL/cntMAX_sencary)+((CENTER_L-(sumNoCL/cntMAX_sencary))*0.25));
	THRESHOLD_R=(int)((sumNoCR/cntMAX_sencary)+((CENTER_R-(sumNoCR/cntMAX_sencary))*0.25));

	irumi_LED();
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			break;
		}
	}
	mpu6500_do_off();
	kasoku(0.0, 500.0, 10000.0, hazimari, 4, 0);
	tohasi_log_flag =1;
	hasira3=tohasi_log;
	gensoku(500.0, 0.0, 10000.0, 90.0, 0, 0);
	wait_ms(200);
	DC_motor_stop();
	for(cnt_sencary=0; cnt_sencary<cntMAX_sencary;cnt_sencary++){
		sumCFL+=g_sensor[0][0];
		sumCFR+=g_sensor[3][0];
		wait_ms(50);
	}
	CENTER_FL=sumCFL/cntMAX_sencary;
	CENTER_FR=sumCFR/cntMAX_sencary;
	irumi_LED();
	wait_ms(1000);

//	tohasira=hasira1;

//	while(1){
//		printf("ready\n\r");
//		while (1) {
//			if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
//				irumi_LED();
//				break;
//			}
//		}
//		printf("CEN_L %d, CEN_R %d\n\r",CENTER_L, CENTER_R);
//		printf("CEN_FL %d, CEN_FR %d, SURA %d\n\r",CENTER_FL, CENTER_FR, SURAKABE_F);
//		printf("Tho_FL %d, Tho_FR %d, Tho_L %d, Tho_R %d\n\r", THRESHOLD_FL, THRESHOLD_FR, THRESHOLD_L, THRESHOLD_R);
//		printf("tohasira1 %f, tohasira2 %f, %f\n\r",hasira1, hasira2, hasira3);
//	}
}

void tes_sendiff2(){
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			break;
		}
	}
	mpu6500_do_off();
	sensor_log_flag = 1;
	Gy_En_log_flag=1;
	sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari+360.0, 0, 0);
	wait_ms(200);
	DC_motor_stop();
	sensor_log_flag = 0;
	Gy_En_log_flag=0;
	irumi_LED();
	while(1){
		printf("ready\n\r");
		while (1) {
			if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
				irumi_LED();
				break;
			}
		}
		display_sensor_log();
	}
}

void tes_sura(int mode_tyouseisura){
	while(1){
		if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
			irumi_LED();
			mpu6500_do_off();
			start_kyuin(20.0);
			wait_ms(700);
			start_kyuin(25.0);
			wait_ms(300);
			start_kyuin(30.0);
			wait_ms(300);
			start_kyuin(35.0);
			wait_ms(1300);

			if(mode_tyouseisura==1){		//探索
//				G_stopPID_flag=1;
				Gy_En_log_flag=1;
//				sakaku_kaiten(0.0, -500.0, 0.0, 5000.0, 90.0*kaitenC);
//				DC_motor_stop();
//				wait_ms(50);
//				sakaku_kaiten(0.0, -500.0, 0.0, 5000.0, 90.0*kaitenC);
//				DC_motor_stop();
//				wait_ms(50);
//				sakaku_kaiten(0.0, -500.0, 0.0, 5000.0, 90.0*kaitenC);
//				DC_motor_stop();
//				wait_ms(50);
//				sakaku_kaiten(0.0, -500.0, 0.0, 5000.0, 90.0*kaitenC);
//				DC_motor_stop();
//				wait_ms(50);

				kasoku(0.0, 500.0, 10000.0, hazimari, 0, 0);
				sinsura(560.0 ,8000.0 ,91.0 ,G_tansakumae ,20.0, 0, 10);
				Gy_En_log_flag=0;
				gensoku(500.0, 0.0, 10000.0, 180.0, 0, 0);
			}else if(mode_tyouseisura==2){					//大廻90
//				kasoku(0.0, 1700.0, 30000.0, hazimari+90);
				Gy_En_log_flag=1;
				kasoku(0.0, 1500.0, 26500.0, hazimari+90.0, 4, 0);
				G_stopPID_flag=1;
				sinsura(1700.0 ,19000.0 ,90 ,19.0 ,58.0, 1, 10);
				Gy_En_log_flag=0;
				gensoku(1500.0, 0.0, 26500.0, 90.0, 0, 0);
			}else if(mode_tyouseisura==3){					//大廻180
				Gy_En_log_flag=1;
				kasoku(0.0, 1500.0, 26500.0, hazimari+90, 4, 0);
				G_stopPID_flag=1;
				sinsura(950.0 ,16000.0 ,180.0 ,20.0 ,60.0, 1, 10);
				Gy_En_log_flag=0;
				gensoku(1500.0, 0.0, 26500.0, 180.0, 0, 0);
			}else if(mode_tyouseisura==4){					//45in吸引
				kasoku(0.0, 1500.0, 25000.0, hazimari+90, 4, 2);
//				kasoku(0.0, 1500.0, 25000.0, hazimari-tohasira);
//				G_stopPID_flag=1;
				Gy_En_log_flag=1;
				sinsura(1200.0 ,45000.0 ,45.0 ,0.0 ,76.0, 3, 10);
				gensoku(1500.0, 0.0, 25000.0, 90.0*1.4141, 0, 0);
				Gy_En_log_flag=0;
			}else if(mode_tyouseisura==5){					//135in吸引
				kasoku(0.0, 1500.0, 25000.0, hazimari+90, 4, 2);
//				kasoku(0.0, 1500.0, 25000.0, hazimari-tohasira);
				G_stopPID_flag=1;
				Gy_En_log_flag=1;
				sinsura(1200.0 ,27500.0 ,135.0 ,26.0 ,54.0, 3, 10);
				Gy_En_log_flag=0;
				gensoku(1500.0, 0.0, 25000.0, 90.0*1.4141, 0, 0);
			}else if(mode_tyouseisura==6){			//45out
				G_state=4;
				kasoku(0.0, 1500.0, 25000.0, 90*1.4141, 3, 3);
//				kasoku(0.0, 1500.0, 25000.0, hazimari-tohasira);
				G_stopPID_flag=1;
				Gy_En_log_flag=1;
				sinsura(1100.0 ,25000.0 ,45.0 ,66.0 ,25.0, 2, 10);
				Gy_En_log_flag=0;
				gensoku(1500.0, 0.0, 25000.0, 90.0, 0, 0);
			}else if(mode_tyouseisura==7){			//135out
				G_state=4;
				kasoku(0.0, 1500.0, 25000.0, 90*1.4141, 3, 3);
//				kasoku(0.0, 1500.0, 25000.0, hazimari-tohasira);
				G_stopPID_flag=1;
				Gy_En_log_flag=1;
				sinsura(1200.0 ,32500.0 ,135.0 ,76.0 ,96.0, 2, 10);
				Gy_En_log_flag=0;
				gensoku(1500.0, 0.0, 25000.0, 90.0, 0, 0);
			}else if(mode_tyouseisura==8){			//v90
				G_state=4;
				kasoku(0.0, 1500.0, 25000.0, 90*1.4141, 3, 3);
//				kasoku(0.0, 1500.0, 25000.0, hazimari-tohasira);
				G_stopPID_flag=1;
				Gy_En_log_flag=1;
				sinsura(1200.0 ,35000.0 ,90.0 ,40.0 ,37.0, 4, 10);
				Gy_En_log_flag=0;
				gensoku(1500.0, 0.0, 25000.0, 90.0*1.4141, 0, 0);
			}else if(mode_tyouseisura==9){		//小回り早いの
				G_saitan_mode=3;
				Gy_En_log_flag=1;
				kasoku(0.0, 1300.0, 25000.0, hazimari+180, 4, 0);
				G_stopPID_flag=1;
				sinsura(1500.0 ,65000.0 ,92.0 ,2.0 ,18.0, 0, 10);
				Gy_En_log_flag=0;
				gensoku(1300.0, 0.0, 26500.0, 180.0, 0, 0);
			}else if(mode_tyouseisura==10){		//なんか使うよう
				if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
	//				G_stopPID_flag=1;
					irumi_LED();
					mpu6500_do_off();
					start_kyuin(20.0);
					wait_ms(1500);
					start_kyuin(25.0);
					wait_ms(500);
					start_kyuin(30.0);
					wait_ms(1000);
					Gy_En_log_flag=1;
	//				kasoku(0.0, 1700.0, 32500.0, (hazimari-tohasira));
					sakaku_daikei(00, 4000, 00, 15000, 540, 0 ,0);
	//				gensoku(1700.0, 0.0, 32500.0, 180.0);
					Gy_En_log_flag=0;
					DC_motor_stop();
					stop_kyuin();
					irumi_LED();
					while(1){
						printf("ready\n\r");
						while (1) {
							if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
								irumi_LED();
								break;
							}
						}
						display_En_Gy();
					}
				}
			}else if(mode_tyouseisura==11){
				G_saitan_mode=3;
				kasoku(0.0, 1200.0, 30000.0, hazimari+180.0, 4, 1);
				G_stopPID_flag=1;
				Gy_En_log_flag=1;
				sinsura(1300.0 ,25000.0 ,89.0 ,9.0 ,3.0, 0, 5);
				Gy_En_log_flag=0;
				gensoku(1200.0, 0.0, 30000.0, 180.0, 0, 0);
			}
			DC_motor_stop();
			stop_kyuin();
			irumi_LED();
			while(1){
				printf("ready\n\r");
				while (1) {
					if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
						irumi_LED();
						break;
					}
				}
				display_En_Gy();
			}
		}
	}
}

void set_sensor(){
	while(1){
		if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
			irumi_LED();
			CENTER_L=g_sensor[1][0];
			CENTER_R=g_sensor[2][0];
			mpu6500_do_off();
			reset_sen_log();
			sensor_log_flag = 1;
			G_stopPID_flag=1;
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, 540.0, 5, 5);
			G_stopPID_flag=0;
			sensor_log_flag = 0;
			DC_motor_stop();
			search_sen(5);
			THRESHOLD_L=G_logminSEN_L+((int)((CENTER_L-G_logminSEN_L)*0.3));
			THRESHOLD_R=G_logminSEN_R+((int)((CENTER_R-G_logminSEN_R)*0.3));
			break;
		}
	}
	irumi_LED();
	while(1){
		if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
			irumi_LED();
			mpu6500_do_off();
			reset_sen_log();
			sensor_log_flag = 1;
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, 360.0, 0, 0);
			DC_motor_stop();
			sensor_log_flag = 0;
			search_sen(2);
			THRESHOLD_DIF_L=G_logmaxdiff_L2*0.8;
//			+((int)((G_logmaxdiff_L2-G_logmaxdiff_L1)*0.5));
			break;
		}
	}
	irumi_LED();
	while(1){
		if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
			irumi_LED();
			mpu6500_do_off();
			reset_sen_log();
			sensor_log_flag = 1;
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, 360.0, 0, 0);
			DC_motor_stop();
			sensor_log_flag = 0;
			search_sen(3);
			THRESHOLD_DIF_R=G_logmaxdiff_R2*0.8;
//			+((int)((G_logmaxdiff_R2-G_logmaxdiff_R1)*0.5));
			break;
		}
	}
	while(1){
		if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
			irumi_LED();
			mpu6500_do_off();
			sensor_log_flag = 1;
			G_stopPID_flag=1;
			reset_sen_log();
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari, 4, 1);
			G_stopPID_flag=0;
			Test_flag=1;
			DC_motor_stop();
			sensor_log_flag = 0;
			search_sen(4);
			THRESHOLD_FL=G_logminSEN_FL+((int)((G_logmaxSEN_FL-G_logminSEN_FL)*0.5));
			THRESHOLD_FR=G_logminSEN_FR+((int)((G_logmaxSEN_FR-G_logminSEN_FR)*0.5));
			break;
		}
	}
	while(1){
		if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
			irumi_LED();
			printf("CEN_L%d, CEN_R%d, THO_L%d, THO_R%d, MINL%d, MINR%d\n\r",CENTER_L, CENTER_R, THRESHOLD_L, THRESHOLD_R, G_logminSEN_L, G_logminSEN_R);
			printf("DIF_L%d, DIF_R%d, DIFL2_%d, DIFR2_%d, DIFL1_%d, DIFR1_%d\n\r",THRESHOLD_DIF_L, THRESHOLD_DIF_R, G_logmaxdiff_L2, G_logmaxdiff_R2, G_logmaxdiff_L1 ,G_logmaxdiff_R1);
			printf("THO_FL%d, THO_FR%d, MINFL%d, MINFR%d, MAXFL%d, MAXFR%d\n\r",THRESHOLD_FL, THRESHOLD_FR, G_logminSEN_FL, G_logminSEN_FR, G_logmaxSEN_FL, G_logmaxSEN_FR);
			display_sensor_log();
		}
	}
}

void adati_test(){
	G_saitan_mode=0;
	int count_tes=0;
	int Tes_number[5];
	irumi_LED();
	mpu6500_do_off();
	G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
	G_look_position_y=0;
	G_look_position_x=0;
	G_direction=1080;

	G_goal_y=4;
	G_goal_x=4;
	mpu6500_do_off();
	Set_Wall();

	while(1){
		Record_Wall();
		pl_hosu_set();
		pl_hosu_make();
		if(G_finish_flag==1){
			G_finish_flag = 0;
			break;
		}
		while(1){
			if((G_X_Motor>19.0)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
				move_adati();
				goal_flag();
				break;
			}
		}
		Tes_number[count_tes]=g_sensor[1][0];
		count_tes++;
		if(count_tes==3){
			sakaku_daikei(500.0, 500.0, 0.0, 10000.0, 180.0, 5, 5);
			DC_motor_stop();
			while(1){
				if((g_sensor[0][0]>300)&&(g_sensor[3][0]>300)){
					display_wallhosu();
					printf("%d, %d, %d, %d, %d\n\r",Test_flag, Tes_number[0], Tes_number[1], Tes_number[2], Tes_number[3]);
				}
			}
		}
	}
}

void flashtest(){
	int i_step_flash;
	FLASH_WALL_DATA flash_data;
	// 1. まず読み出す
	display_wallhosu();
	LoadFlashWall(&flash_data);
	Set_Wall2();
	// 2. データを書き換える（例）
//	CENTER_R = flash_data.sensor.CENTER_R;
	for(i_step_flash=0;i_step_flash<G_maxMAP+1;i_step_flash++){
		flash_data.Row[i_step_flash] = G_Row[i_step_flash];
		flash_data.Column[i_step_flash] = G_Column[i_step_flash];
	}
	display_wallhosu();
	// 3. フラッシュに保存（セクタ全体が更新される）
	SaveFlashWall(&flash_data);
}

void flashtest2(){
	int i_step_flash;
	display_wallhosu();
	printf("%d,%d\r\n",CENTER_L,CENTER_R);
	Load_wall();
	Load_sensor();
	printf("%d,%d\n\r",CENTER_L,CENTER_R);
	display_wallhosu();
	// 3. フラッシュに保存（セクタ全体が更新される）
	Save_wall();
	Save_sensor();
}

void tes_sandai(float wVm, float wAmax, float wXall, int a_step){
	//mode_sura 0探索，1st_st, 2na_st, 3st_na, 4na_na
	float wv1_sura, wv2_sura;
	float acc_sura, acc_sura2, sub_acc, wxall_sura;
	int kk2, k2_sura, k2_sura2;
	float ca1,ca2,ca3;
	int a_step2;
	acc_sura=wAmax/a_step;
	acc_sura2=acc_sura;
	a_step2=a_step;

	wv1_sura=acc_sura*0.5*0.001*(a_step-1)*(a_step);
	wv2_sura=(wVm-(wv1_sura*2));
	k2_sura=wv2_sura/(wAmax*0.001);
	k2_sura2=k2_sura;
	if(wv2_sura!=k2_sura*(wAmax*0.001)){
		a_step2=a_step+1;
		k2_sura2=k2_sura-1;
		wv1_sura=wv1_sura+(wAmax*0.001);
		wv2_sura=wAmax*0.001*k2_sura2;
		acc_sura2=acc_sura*((wVm-wv2_sura)/(2*wv1_sura));
	}
	wxall_sura=wVm*(2*(a_step2-1)+k2_sura2)*0.001;
//	printf("hello %f\n\r",wxall_sura);
	ca1=1/wAmax;
	ca2=0.002*(a_step2-1)-(2*wv1_sura/wAmax);
	if(wVm>0.0){
		if(wXall-5.0<=wxall_sura){
			ca3=-(wXall-5);
			wVm=(-ca2+sqrt((ca2*ca2)-(4*ca1*ca3)))/(2*ca1);
		}
	}else{
		if(wXall+5.0>wxall_sura){
			ca3=-(wXall+5);
			wVm=(-ca2+sqrt((ca2*ca2)-(4*ca1*ca3)))/(2*ca1);
		}
	}
	wv1_sura=acc_sura*0.5*0.001*(a_step-1)*(a_step);
	wv2_sura=(wVm-(wv1_sura*2));
	k2_sura=wv2_sura/(wAmax*0.001);
	k2_sura2=k2_sura;
	if(wv2_sura!=k2_sura*(wAmax*0.001)){
		a_step2=a_step+1;
		k2_sura2=k2_sura-1;
		wv1_sura=wv1_sura+(wAmax*0.001);
		wv2_sura=wAmax*0.001*k2_sura2;
		acc_sura2=acc_sura*((wVm-wv2_sura)/(2*wv1_sura));
	}
	wxall_sura=wVm*(2*(a_step2-1)+k2_sura2)*0.001;
	printf("hello %f\n\r",wxall_sura);
	sub_acc=wAmax-(a_step2-1)*acc_sura2;
	kk2=((wXall-wxall_sura)/(wVm*0.001))+1;
	Gy_En_log_flag=1;
	while(1){
		G_wXturn=0.0;
		G_waturn=acc_sura2;
		sumGyro=0.0;
		G_timer=0;
		while(1){
			if(G_timer==a_step2-1){
				G_wAturn=wAmax;
				G_waturn=0.0;
				break;
			}
		}
		G_timer=0;
		while(1){
			if(G_timer>=k2_sura2){
				G_waturn=-sub_acc;
				break;
			}
		}
		G_timer=0;
		while(1){
			if(G_timer==1){
				G_waturn=-acc_sura2;
			}
			if(G_timer==a_step2-1){
				G_waturn=0.0;
				G_wAturn=0.0;
				break;
			}
		}
		G_timer=0;
		while(1){
			if(G_timer>=kk2){
				G_waturn=-acc_sura2;
				break;
			}
		}
		G_timer=0;
		while(1){
			if(G_timer>=a_step2-1){
				G_waturn=0.0;
				G_wAturn=-wAmax;
				break;
			}
		}
		G_timer=0;
		while(1){
			if(G_timer>=k2_sura2){
				G_waturn=sub_acc;
				break;
			}
		}
		G_timer=0;
		while(1){
			if(G_timer==1){
				G_waturn=acc_sura2;
			}
			if(G_timer==a_step2-1){
				G_waturn=0.0;
				G_wAturn=0.0;
				G_wXturn=0.0;
				sumGyro=0.0;
				break;
			}
		}
		break;
	}
	Gy_En_log_flag=0;
	while(1){
		printf("ready\n\r");
		while (1) {
			if (g_sensor[0][0] >= 300 && g_sensor[3][0] >= 300) {
				irumi_LED();
				break;
			}
		}
		display_En_Gy();
	}
}
