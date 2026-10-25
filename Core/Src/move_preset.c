/*
 * move_preset.c
 *
 *  Created on: Oct 6, 2025
 *      Author: shino
 */

#include "move_preset.h"

#include "tim.h"
#include "PL_gyro.h"
#include "PL_timer.h"
#include "PL_sensor.h"
#include "motor_seigyo.h"
#include "Wall_Control.h"
#include "Wall_record.h"
#include "Set_goal.h"
#include "Hosu_make.h"

#include "G_hensu.h"

void DC_motor_stop(){
	wait_ms(100);
	G_state=0;
}

void turn_90R(){
	sakaku_kaiten(500.0, 8000.0, 95.0, 15);
	DC_motor_stop();
	G_direction+=90;
}

void turn_90L(){
	sakaku_kaiten(-500.0, -8000.0, -95.0, 15);
	DC_motor_stop();
	G_direction-=90;
}

void turn_180R(){
	 turn_90R();
	 wait_ms(50);
	 turn_90R();
}

void move_back(){
	sakaku_daikei(0.0, -300.0, 0.0, 1000.0, 200.0, 5, 5);
}

void move_start(){
	G_stopPID_flag=1;
	turn_90R();

	move_back();
	DC_motor_stop();
	wait_ms(100);

	sakaku_daikei(0.0, 500.0, 0.0, 5000.0, hazimari-90.0, 5, 5);
	DC_motor_stop();
	wait_ms(100);

	turn_90L();

	move_back();
	DC_motor_stop();
	wait_ms(100);
	G_stopPID_flag=0;
}

void move_start2(){
	sakaku_daikei(500.0, 500.0, 0.0, 10000.0, 90.0, 1, 0);
	DC_motor_stop();
	wait_ms(100);

	G_stopPID_flag=1;
	turn_90L();

	move_back();
	DC_motor_stop();
	wait_ms(100);

	sakaku_daikei(0.0, 500.0, 0.0, 5000.0, hazimari-90.0, 5, 5);
	DC_motor_stop();
	wait_ms(100);

	turn_90L();

	move_back();
	DC_motor_stop();
	G_stopPID_flag=0;

}

void move_gensku(){
	gensoku_stop(10000);
	DC_motor_stop();
}

void move_kabeawase(){
	if(g_sensor[0][5]+g_sensor[3][5]>=SURAKABE_F){
		G_state=10;
		wait_ms(250);
		G_state=11;
		wait_ms(250);
		G_state=12;
		wait_ms(250);
		G_state=0;
		wait_ms(50);
	}
}

void daikusutora(){
	gensoku(500.0, 0.0, 10000.0, 90.0, 1, 2);
	DC_motor_stop();

	G_stopPID_flag=1;
	mpu6500_do_off();
	if(G_kabeL_flag==1){
		turn_90R();
		move_back();
	}else if(G_kabeR_flag==1){
		turn_90L();
		move_back();
	}else if(G_kabeFL_flag==1&&G_kabeFR_flag==1){
		turn_180R();
		move_back();
	}
	DC_motor_stop();
	G_stopPID_flag=0;
	G_kabeate_flag=1;

	daiku_hosumake(0);
	set_hosu_dai();
	set_goal_dai();

	mpu6500_do_off();
}

void move_kabestop(){
	if(G_V_Motor!=0){
		gensoku(G_V_comtur, 0.0, G_A_comst, 90.0, 1, 2);
		DC_motor_stop();

		if(G_kabeF_flag==1){
			move_kabeawase();
		}else if(G_kabeL_flag==1){
			turn_90L();
			move_kabeawase();
		}else if(G_kabeR_flag==1){
			turn_90R();
			move_kabeawase();
		}
	}
}

void daikusutora2(){
	move_kabestop();
	daiku_hosumake(0);
	set_hosu_dai();
	set_goal_dai();

	G_recostop_flag=1;
	mpu6500_do_off();
}

void start_kyuin(float duty){
	int duty_Fun=(int)((7.4/g_V_batt)*duty);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_Fun);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
	G_kyuin_flag=1;
}
void stop_kyuin(){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0.0);
	__HAL_TIM_SET_AUTORELOAD(&htim2, 100);
	G_kyuin_flag=0;
	wait_ms(1000);
}

