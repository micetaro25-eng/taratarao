/*
 * Wall_Control.c
 *
 *  Created on: Oct 28, 2025
 *      Author: shino
 */
#include "Wall_Control.h"

#include "gpio.h"//sensorLED用

#include "PL_sensor.h"

#include "G_hensu.h"

int G_kabeFL_flag;//
int G_kabeFR_flag;//
int G_kabeF_flag;//
int G_kabeL_flag;//
int G_kabeR_flag;//

int G_kabekireL_flag=0;//
int G_kabekireR_flag=0;//

float G_subX_suraL=0;//
float G_subX_suraR=0;//

int subkireL_flag;
int subkireR_flag;

int g_sensor_diff_L=0;
int g_sensor_diff_R=0;
int g_sensor_diff_Lna=0;
int g_sensor_diff_Rna=0;

int fast_kabekire=1;

float subX_diff=0;

int G_WallControlStatus;
int G_WallState;

void judge_kabe(){
	int set_L, set_R;
	if(G_state==1||G_state==2||G_state==4||G_state>=10){
		if(G_state==1||G_state==2||G_state>=10){
			set_L=1;
			set_R=2;
			if(g_sensor[0][0]>=THRESHOLD_FL){
				G_kabeFL_flag=1;
			}else{
				G_kabeFL_flag=0;
			}
			if(g_sensor[3][0]>=THRESHOLD_FR){
				G_kabeFR_flag=1;
			}else{
				G_kabeFR_flag=0;
			}
			if(g_sensor[0][0]+g_sensor[3][0]>=THRESHOLD_FL+THRESHOLD_FR){
				G_kabeF_flag=1;
			}else{
				G_kabeF_flag=0;
			}
		}else{
			set_L=4;
			set_R=5;
			if(g_sensor[0][0]>=THRESHOLD_naFL){
				G_kabeFL_flag=1;
			}else{
				G_kabeFL_flag=0;
			}
			if(g_sensor[3][0]>=THRESHOLD_naFR){
				G_kabeFR_flag=1;
			}else{
				G_kabeFR_flag=0;
			}
		}

		if(g_sensor[set_L][0]>=THRESHOLD_L){
			G_kabeL_flag=1;
			subkireL_flag=1;
		}else{
			G_kabeL_flag=0;
		}
		if(g_sensor[set_R][0]>=THRESHOLD_R){
			G_kabeR_flag=1;
			subkireR_flag=1;
		}else{
			G_kabeR_flag=0;
		}
	}else{
		G_kabeFL_flag=0;
		G_kabeFR_flag=0;
		G_kabeF_flag=0;
		G_kabeL_flag=0;
		G_kabeR_flag=0;
		subkireL_flag=0;
		subkireR_flag=0;
	}
}

void sensor_diff(){
	//sensor_diff 及びターン時のリセット
	int setL, setR;
	int diffL, diffR;
	int max_Lx, max_Rx;
	if((G_state==1)||(G_state==2)||(G_state==4)){
		subX_diff = subX_diff + G_V_Motor * 0.001;
		if(subX_diff>=2.5){
			if(G_state==1||G_state==2){
				setL=1;
				setR=2;
			}else if(G_state==4){
				setL=4;
				setR=5;
			}
			g_sensor_diff_L=g_sensor[setL][0]-g_sensor[setL][5];
			g_sensor_diff_R=g_sensor[setR][0]-g_sensor[setR][5];

			if(fast_kabekire==1){
				g_sensor_diff_L=0;
				g_sensor_diff_R=0;
				fast_kabekire=0;
			}

			g_sensor[setL][5]=g_sensor[setL][0];
			g_sensor[setR][5]=g_sensor[setR][0];

			subX_diff=0;

	//壁キレの判断
			if(G_state==1||G_state==2){
				diffL=THRESHOLD_DIF_L;
				diffR=THRESHOLD_DIF_R;
			}else{
				diffL=THRESHOLD_DIF_naL;
				diffR=THRESHOLD_DIF_naR;
			}

			if(g_sensor_diff_L<=(0-diffL)){
				G_kabekireL_flag=1;
			}
			if(g_sensor_diff_R<=(0-diffR)){
				G_kabekireR_flag=1;
			}

			if((subkireL_flag==1)&&(G_kabeL_flag==0)){
				subkireL_flag=0;
				G_kabekireL_flag=1;
			}
			if((subkireR_flag==1)&&(G_kabeR_flag==0)){
				subkireR_flag=0;
				G_kabekireR_flag=1;
			}
		}
	}else{
		subX_diff=0;
		G_subX_suraL=0;
		G_subX_suraR=0;
		G_kabekireL_flag=0;
		G_kabekireR_flag=0;
		fast_kabekire=1;
	}

	//壁切れ計算開始
	if(G_kabekireL_flag==1){
		G_subX_suraL = G_subX_suraL + G_V_Motor * 0.001;
	}
	if(G_kabekireR_flag==1){
		G_subX_suraR = G_subX_suraR + G_V_Motor * 0.001;
	}

	//壁切れリセット
	if(G_state==1||G_state==2){
		max_Lx=120;		//普通のスラローム67.5+20< <67.5+90壁が2個なく先で大廻するとき
		max_Rx=120;
	}else{
		max_Lx=110;		//普通のスラローム10+前距離(70,80)< <10+90*1.414(127.3)
		max_Rx=110;
	}

	if(G_subX_suraL>=max_Lx){
		G_subX_suraL=0;
		G_kabekireL_flag=0;
	}
	if(G_subX_suraR>=max_Rx){
		G_subX_suraR=0;
		G_kabekireR_flag=0;
	}
}

void kensa_wall_judge(){
	if(G_kabeF_flag==0){
		if ((G_kabeL_flag==0)&&(G_kabeR_flag==0)){
			G_WallState=0;
		}else if(G_kabeL_flag==0){
			G_WallState=1;
		}else if(G_kabeR_flag==0){
			G_WallState=2;
		}else{
			G_WallState=3;
		}
	}else{
		if ((G_kabeL_flag==0)&&(G_kabeR_flag==0)){
			G_WallState=4;
		}else if(g_sensor[1][0]<=1.2*THRESHOLD_L){
			G_WallState=5;
		}else if(g_sensor[2][0]<=1.2*THRESHOLD_R){
			G_WallState=6;
		}else{
			G_WallState=7;
		}
	}
}


//void PID_wall_judge(){
//	if ((G_kabeL_flag==0||G_kabekireL_flag==1)
//		&&(G_kabeR_flag==0||G_kabekireR_flag==1)){
//		G_WallControlStatus=0;
//	}else if((G_kabeL_flag==0||G_kabekireL_flag==1)){
//		G_WallControlStatus=1;
//	}else if((G_kabeR_flag==0||G_kabekireR_flag==1)){
//		G_WallControlStatus=2;
//	}else{
//		G_WallControlStatus=3;
//	}
//}

void check_wall(){
	if(G_LED_mode==2){
		if(G_kabeL_flag==1){
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}
		if(G_kabeR_flag==1){
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_RESET);
		}
		if(G_kabeFL_flag==1){
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		}
		if(G_kabeFR_flag==1){
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		}else{
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		}
		if(G_kabekireL_flag==1){
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}else{
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		}
		if(G_kabekireR_flag==1){
			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_SET);
		}else{
//			HAL_GPIO_WritePin(LED7_GPIO_Port, LED7_Pin, GPIO_PIN_RESET);
		}
	}
}
