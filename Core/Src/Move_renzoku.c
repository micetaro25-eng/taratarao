/*
 * Move_renzoku.c
 *
 *  Created on: Oct 8, 2025
 *      Author: shino
 */

#include "Move_renzoku.h"
#include "gpio.h"
#include "stdio.h"
#include "math.h"

#include <PL_timer.h>
#include "PL_gyro.h"

#include "motor_seigyo.h"
#include "Wall_Control.h"
#include "Wall_record.h"
#include "Moveonly_sura.h"
#include "move_preset.h"

#include "G_hensu.h"

void ren_move_st(){
	if(G_V_Motor==0){
		if(G_kabeate_flag==1){
			G_kabeate_flag=0;
			kasoku(0.0, G_V_comtur, G_A_comst, hazimari, 4, 1);
		}else{
			kasoku(0.0, G_V_comtur, G_A_comst, 180.0, 1, 1);
		}
	}else{
		renStr(160.0, 6, 1);
	}
}

void ren_move_L(){
	if(G_V_Motor==0){
		if(G_kabeate_flag==1){
			G_kabeate_flag=0;
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari-90.0, 4, 2);
		}else{
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, 90.0, 1, 2);
		}
		turn_90L();
		kasoku(0.0, G_V_comtur, G_A_comst, 90.0 ,2 ,1);
	}else{
		sura_moveL();
	}
}

void ren_move_R(){
	if(G_V_Motor==0){
		if(G_kabeate_flag==1){
			G_kabeate_flag=0;
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari-90.0, 4, 2);
		}else{
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, 90.0, 1, 2);
		}
		turn_90R();
		kasoku(0.0, G_V_comtur, G_A_comst, 90.0, 2, 1);
	}else{
		sura_moveR();
	}
}

void ren_move_turn(){
	kensa_wall_judge();
	if(G_WallState>=4){
		gensoku(G_V_comtur, 0.0, G_A_comst, 70.0, 1, 2);
		DC_motor_stop();
		mpu6500_do_off();
		if(G_WallState==6||G_WallState==7){
			turn_90R();
		}
		if(G_WallState==5){
			turn_90L();
		}
		if(G_WallState>=5){
			move_back();
			DC_motor_stop();

			sakaku_daikei(0.0, 500.0, 0.0, 5000.0, 50.0, 1, 0);
			DC_motor_stop();

			if(G_WallState==6||G_WallState==7){
				turn_90R();
			}
			if(G_WallState==5){
				turn_90L();
			}
		}else{
			turn_180R();
			DC_motor_stop();
		}
		move_back();
		DC_motor_stop();
		mpu6500_do_off();
		G_kabeate_flag=1;
	}else{
		gensoku(G_V_comtur, 0.0, G_A_comst, 160.0, 1, 1);
		DC_motor_stop();
		mpu6500_do_off();

		turn_180R();
		DC_motor_stop();

		mpu6500_do_off();
	}
}

void ren_move_st2(){
	if(G_V_Motor==0){
		if(G_start_flag==1){
			kasoku(0.0, G_V_comtur, G_A_comst, 90.0, 2, 1);
			G_recostop_flag=0;
		}else{
			G_start_flag=1;
			kasoku(0.0, G_V_comtur, G_A_comst, hazimari, 4, 1);
			G_recostop_flag=0;
		}
	}else{
		renStr(160.0, 6, 1);
	}
}

void ren_move_L2(){
	if(G_V_Motor==0){
		turn_90L();
		kasoku(0.0, G_V_comtur, G_A_comst, 90.0, 2, 0);
		G_recostop_flag=0;
	}else{
		sura_moveL();
	}
}

void ren_move_R2(){
	if(G_V_Motor==0){
		turn_90R();
		kasoku(0.0, G_V_comtur, G_A_comst, 90.0, 2, 0);
		G_recostop_flag=0;
	}else{
		sura_moveR();
	}
}

void ren_move_turn2(){
	if(G_kabeF_flag==1){
		if(G_kabeR_flag==1){
			gensoku(G_V_comtur, 0.0, G_A_comst, 70.0, 6, 0);
			move_kabeawase();
			turn_90R();
			move_kabeawase();
			turn_90R();
		}else if(G_kabeL_flag==1){
			gensoku(G_V_comtur, 0.0, G_A_comst, 70.0, 6, 0);
			move_kabeawase();
			turn_90L();
			move_kabeawase();
			turn_90L();
		}
	}else{
		if(G_kabeR_flag==1){
			gensoku(G_V_comtur, 0.0, G_A_comst, 70.0, 6, 0);
			turn_90R();
			move_kabeawase();
			turn_90R();
		}else if(G_kabeL_flag==1){
			gensoku(G_V_comtur, 0.0, G_A_comst, 70.0, 6, 0);
			turn_90L();
			move_kabeawase();
			turn_90L();
		}else{
			gensoku(G_V_comtur, 0.0, G_A_comst, 70.0, 6, 0);
			turn_180R();
		}
	}
	mpu6500_do_off();
	G_recostop_flag=1;
}
