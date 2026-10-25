/*
 * Record_something.c
 *
 *  Created on: Oct 15, 2025
 *      Author: shino
 */

#include "Record_something.h"

#include "stdio.h"
#include "math.h"
#include "usart.h"

#include "PL_sensor.h"
#include "Wall_Control.h"
#include "motor_seigyo.h"
#include "G_hensu.h"

int i_log=0, j_log=0;
int j_log_sign1=0, j_log_sign2=0;
int do_logsign_flag=0;

int RecodeL[1000];
int RecodeR[1000];
int RecodeFL[1000];
int RecodeFR[1000];

float Recode_Gy[1000];
float Recode_Gw[1000];
float Recode_En[1000];
float Recode_GV[1000];
float Recode_DuL[1000];
float Recode_DuR[1000];

int sensor_log_flag=0;
int sensordiff_log_flag=0;


void sensor_log(){
	i_log++;
	if(i_log%5==0){
		RecodeL[j_log]=g_sensor[1][0];
		RecodeR[j_log]=g_sensor[2][0];
		RecodeFL[j_log]=g_sensor[0][0];
		RecodeFR[j_log]=g_sensor[3][0];
		if(do_logsign_flag==0){
			if(G_kabekireL_flag==1||G_kabekireR_flag==1){
				do_logsign_flag=1;
				j_log_sign1=j_log;
			}
		}
		if(Test_flag==1){
			j_log_sign2=j_log;
		}
		j_log++;
	}
}

void reset_sen_log(){
	i_log=0;
	while(1){
		if(i_log==j_log+1){
			break;
		}
		RecodeR[i_log]=0;
		RecodeL[i_log]=0;
		RecodeFR[i_log]=0;
		RecodeFL[i_log]=0;
		i_log++;
	}
	i_log=0;
	j_log=0;
}

void log_Enc_Gyro(){
//	i_log++;
//	if(i_log%10==0){
		Recode_Gy[j_log]=Gyro[0];
		Recode_Gw[j_log]=G_wVturn;
		Recode_En[j_log]=(Enc_L[0]+Enc_R[0])*0.5;
		Recode_GV[j_log]=G_V_Motor;
		Recode_DuL[j_log]=G_duty_L;
		Recode_DuR[j_log]=G_duty_R;
		j_log++;
//	}
}

void display_En_Gy(){
	i_log=0;
	while(1){
		if(j_log==i_log+1){
			break;
		}
		printf("%f, %f, %f, %f, %f, %f\n\r",Recode_Gy[i_log],Recode_Gw[i_log],Recode_En[i_log],Recode_GV[i_log],Recode_DuL[i_log],Recode_DuR[i_log]);
		i_log++;
	}
}

void display_sensor_log(){
	i_log=0;

	while(1){
		printf("SENL=%d SENR=%d SENFL=%d SENFR=%d\n\r",RecodeL[i_log] ,RecodeR[i_log] ,RecodeFL[i_log] ,RecodeFR[i_log]);
		i_log++;
		if(i_log==j_log){
			break;
		}
	}
	//2倍ケア
}

void search_sen(int setsen_mode){
	i_log=0;//2倍ケア
	if(setsen_mode==1||setsen_mode==5){
		G_logminSEN_L=RecodeL[0];
		G_logminSEN_R=RecodeR[0];
	}
	if(setsen_mode==2||setsen_mode==5){
		G_logmaxdiff_L1=RecodeL[1]-RecodeL[0];
		G_logmaxdiff_L2=RecodeL[1800/25]-RecodeL[(1800/25)-1];
	}
	if(setsen_mode==3||setsen_mode==5){
		G_logmaxdiff_R1=RecodeR[1]-RecodeR[0];
		G_logmaxdiff_R2=RecodeR[1800/25]-RecodeR[(1800/25)-1];
	}
	while(1){
		i_log++;
		if(i_log==j_log){
			break;
		}
		if(setsen_mode==1||setsen_mode==5){
			if(RecodeL[i_log]<G_logminSEN_L){
				G_logminSEN_L=RecodeL[i_log];
			}
			if(RecodeR[i_log]<G_logminSEN_R){
				G_logminSEN_R=RecodeR[i_log];
			}
			if(RecodeFL[i_log]<G_logminSEN_FL){
				G_logminSEN_FL=RecodeFL[i_log];
			}
			if(RecodeFR[i_log]<G_logminSEN_FR){
				G_logminSEN_FR=RecodeFR[i_log];
			}
		}
		if(setsen_mode==2||setsen_mode==5){
			if((i_log>=1)&&(i_log<=(1800/25))){
				if(RecodeL[i_log]-RecodeL[i_log-1]<G_logmaxdiff_L1){
					G_logmaxdiff_L1=RecodeL[i_log]-RecodeL[i_log-1];
				}
			}else{
				if(RecodeL[i_log]-RecodeL[i_log-1]<G_logmaxdiff_L2){
					G_logmaxdiff_L2=RecodeL[i_log]-RecodeL[i_log-1];
				}
			}
		}
		if(setsen_mode==3||setsen_mode==5){
			if((i_log>=1)&&(i_log<=(1800/25))){
				if(RecodeR[i_log]-RecodeR[i_log-1]<G_logmaxdiff_R1){
					G_logmaxdiff_R1=RecodeR[i_log]-RecodeR[i_log-1];
				}
			}else{
				if(RecodeR[i_log]-RecodeR[i_log-1]<G_logmaxdiff_R2){
					G_logmaxdiff_R2=RecodeR[i_log]-RecodeR[i_log-1];
				}
			}
		}
		if(setsen_mode==4){
			if(i_log==j_log_sign1){
				G_logminSEN_FL=RecodeFL[i_log];
				G_logminSEN_FR=RecodeFR[i_log];
			}
			if(i_log==j_log_sign2){
				G_logmaxSEN_FL=RecodeFL[i_log];
				G_logmaxSEN_FR=RecodeFR[i_log];
			}
		}
	}
}
