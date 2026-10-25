/*
 * Adaitimethod.c
 *
 *  Created on: Oct 7, 2025
 *      Author: shino
 */

#include "Adatimethod.h"

#include "math.h"
#include "stdio.h"
#include "gpio.h"

#include "PL_sensor.h"
#include "PL_timer.h"
#include "PL_gyro.h"
//#include "Wall_Control.h"
#include "move_preset.h"
#include "Wall_record.h"
#include "Set_goal.h"
#include "Hosu_make.h"
#include "motor_seigyo.h"
#include "Move_renzoku.h"
#include "Saitan.h"

#include "test_something.h"
#include "PL_flash.h"
#include "G_hensu.h"

int kiti_flag=0;

void move_adati(){
	if(G_direction%360==0){
		if((G_look_position_y!=15)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_st();
			G_look_position_y++;
		}else if((G_look_position_x!=15)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_R();
			G_look_position_x++;
		}else if((G_look_position_x!=0)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_L();
			G_look_position_x--;
		}else if(G_look_position_y!=0&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1]){
			ren_move_turn();
		}else{
			DC_motor_stop();
		}
	}else if(G_direction%360==90){
		if((G_look_position_x!=15)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_st();
			G_look_position_x++;
		}else if((G_look_position_y!=0)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_R();
			G_look_position_y--;
		}else if((G_look_position_y!=15)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_L();
			G_look_position_y++;
		}else if(G_look_position_x!=0&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y]){
			ren_move_turn();
		}else{
			DC_motor_stop();
		}
	}else if(G_direction%360==180){
		if((G_look_position_y!=0)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_st();
			G_look_position_y--;
		}else if((G_look_position_x!=0)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_R();
			G_look_position_x--;
		}else if((G_look_position_x!=15)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_L();
			G_look_position_x++;
		}else if(G_look_position_y!=15&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1]){
			ren_move_turn();
		}else{
			DC_motor_stop();
		}
	}else{
		if((G_look_position_x!=0)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_st();
			G_look_position_x--;
		}else if((G_look_position_y!=0)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_L();
			G_look_position_y--;
		}else if((G_look_position_y!=15)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_R();
			G_look_position_y++;
		}else if(G_look_position_x!=15&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y]){
			ren_move_turn();
		}else{
			DC_motor_stop();
		}
	}
}

void move_adati2(){
	if(G_direction%360==0){
		if(((G_look_position_y+1)<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_st2();
			G_look_position_y++;
		}else if(((G_look_position_x+1)<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_R2();
			G_look_position_x++;
		}else if((0<=(G_look_position_x-1))
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_L2();
			G_look_position_x--;
		}else if((0<=(G_look_position_y-1))
			&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1]){
			ren_move_turn2();
		}else{
			DC_motor_stop();
		}
	}else if(G_direction%360==90){
		if(((G_look_position_x+1)<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_st2();
			G_look_position_x++;
		}else if((0<=(G_look_position_y-1))
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_R2();
			G_look_position_y--;
		}else if(((G_look_position_y+1)<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_L2();
			G_look_position_y++;
		}else if((0<=(G_look_position_x-1))
			&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y]){
			ren_move_turn2();
		}else{
			DC_motor_stop();
		}
	}else if(G_direction%360==180){
		if((0<=(G_look_position_y-1))
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_st2();
			G_look_position_y--;
		}else if((0<=(G_look_position_x-1))
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_R2();
			G_look_position_x--;
		}else if(((G_look_position_x+1)<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_L2();
			G_look_position_x++;
		}else if(((G_look_position_y+1)<G_maxMAP)
			&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1]){
			ren_move_turn2();
		}else{
			DC_motor_stop();
		}
	}else{
		if((0<=(G_look_position_x-1))
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			ren_move_st2();
			G_look_position_x--;
		}else if((0<=(G_look_position_y-1))
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_L2();
			G_look_position_y--;
		}else if(((G_look_position_y+1)<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			ren_move_R2();
			G_look_position_y++;
		}else if(((G_look_position_x+1)<G_maxMAP)
			&&G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y]){
			ren_move_turn2();
		}else{
			DC_motor_stop();
		}
	}
}

void kiti_check(){
	if(G_direction%360==0){
		if((G_look_position_y+1<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y+1])
			&&(G_Row[G_look_position_y+1]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			if((G_look_position_y+2<G_maxMAP)
				&&(G_dist[G_look_position_x][G_look_position_y+1]>G_dist[G_look_position_x][G_look_position_y+2])
				&&((G_wall_recoder_flag[G_look_position_y+1]&(1<<G_look_position_x))==(1<<G_look_position_x))){
				kiti_flag=1;
			}
		}
	}else if(G_direction%360==90){
		if((G_look_position_x+1<G_maxMAP)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x+1][G_look_position_y])
			&&(G_Column[G_look_position_x+1]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			if((G_look_position_x+2<G_maxMAP)
				&&(G_dist[G_look_position_x+1][G_look_position_y]>G_dist[G_look_position_x+2][G_look_position_y])
				&&((G_wall_recoder_flag[G_look_position_y]&(1<<(G_look_position_x+1)))==(1<<(G_look_position_x+1)))){
				kiti_flag=1;
			}
		}
	}else if(G_direction%360==180){
		if((0<=G_look_position_y-1)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x][G_look_position_y-1])
			&&(G_Row[G_look_position_y]&(1<<G_look_position_x))!=(1<<G_look_position_x)){
			if((0<=G_look_position_y-2)
				&&(G_dist[G_look_position_x][G_look_position_y-1]>G_dist[G_look_position_x][G_look_position_y-2])
				&&((G_wall_recoder_flag[G_look_position_y-1]&(1<<G_look_position_x))==(1<<G_look_position_x))){
				kiti_flag=1;
			}
		}
	}else{
		if((0<=G_look_position_x-1)
			&&(G_dist[G_look_position_x][G_look_position_y]>G_dist[G_look_position_x-1][G_look_position_y])
			&&(G_Column[G_look_position_x]&(1<<G_look_position_y))!=(1<<G_look_position_y)){
			if((0<=G_look_position_x-2)
				&&(G_dist[G_look_position_x-1][G_look_position_y]>G_dist[G_look_position_x-2][G_look_position_y])
				&&((G_wall_recoder_flag[G_look_position_y]&(1<<(G_look_position_x-1)))==(1<<(G_look_position_x-1)))){
				kiti_flag=1;
			}
		}
	}
}

void goal_flag(){
	if(G_look_position_x==G_goalpass_x[G_goal_number]&&G_look_position_y==G_goalpass_y[G_goal_number]){
		G_finish_flag=1;
	}
}



//void Adatimethod(int katamiti_flag){
//	G_kabeate_flag=1;
//	G_look_position_y=0;
//	G_look_position_x=0;
//	G_direction=1080;
//	G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
//
//	Set_Wall();
//	mpu6500_do_off();
//	G_fix_mode1=1;
//	G_fix_mode2=1;
//	while(1){
//		while(1){
//			if(G_finish_flag==1){
//				G_finish_flag = 0;
//				break;
//			}
//			pl_hosu_set();
//			pl_hosu_make();
//			while(1){
//				if((G_V_Motor==0)||(G_X_Motor>20.0)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
//					move_adati();
//					Record_Wall();
//					goal_flag();
//					break;
//				}
//			}
//		}
//		if(G_goalpass_x[G_goal_number]==0&&G_goalpass_y[G_goal_number]==0){
//			move_start2();
//			break;
//		}
//		if(katamiti_flag==1){
//			gensoku(G_V_comtur, 0.0, G_A_comst, 70.0);
//			G_look_position_x=0;
//			G_look_position_y=0;
//			G_direction=1080;
//			DC_motor_stop();
//			break;
//		}
//		G_goal_number++;
//	}
//	wait_ms(500);
//}

//void Adatimethod(){
//	Load_wall();
//	Load_sensor();
//	G_start_flag=0;
//	G_finish_flag=0;
//	G_recostop_flag=0;
//
//	mpu6500_do_off();
//	G_look_position_y=0;
//	G_look_position_x=0;
//	G_direction=1080;
//	G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
//	while(1){
//		while(1){
//			goal_flag();
//			if(G_finish_flag==1){
//				G_finish_flag = 0;
//				move_kabestop();
//				Save_wall();
//				Save_Wall();
//				irumi_LED();
//				break;
//			}
//			pl_hosu_set();
//			pl_hosu_make();
//			while(1){
//				if(kiti_flag==0){
//					if((G_V_Motor==0.0)||(G_X_Motor>20.0)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
//						move_adati2();
//						if(G_recostop_flag!=1){
//							Record_Wall();
//						}
//						break;
//					}
//				}
//			}
//		}
//		if(G_goalpass_x[G_goal_number]==0&&G_goalpass_y[G_goal_number]==0){
//			gensoku(G_V_comtur, 0.0, G_A_comst, 90.0, 1, 2);
//			move_kabeawase();
//			turn_180R();
//			G_start_flag=0;
//			Save_wall();
//			Save_Wall();
//			break;
//		}
//		G_goal_number++;
//	}
//	wait_ms(500);
//}

void Adatimethod(){
	Load_wall();
	Load_sensor();
	G_start_flag=0;
	G_finish_flag=0;
	G_recostop_flag=0;

	mpu6500_do_off();
	G_look_position_y=0;
	G_look_position_x=0;
	G_direction=1080;
	G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
	while(1){
		while(1){
			goal_flag();
			if(G_finish_flag==1){
				G_finish_flag = 0;
				if(G_goalpass_x[G_goal_number]==G_firstgoal_x&&G_goalpass_y[G_goal_number]==G_firstgoal_y){
					move_kabestop();
					Save_wall();
					Save_Wall();
					irumi_LED();
				}
				break;
			}
			pl_hosu_set();
			pl_hosu_make();
			while(1){
				if((G_V_Motor==0.0)||(G_X_Motor>G_tansakumae)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
					move_adati2();
					if(G_recostop_flag!=1){
						Record_Wall();
					}
					break;
				}
			}
		}
		if(G_goalpass_x[G_goal_number]==0&&G_goalpass_y[G_goal_number]==0){
			gensoku(G_V_comtur, 0.0, G_A_comst, 90.0, 1, 2);
			move_kabeawase();
			turn_180R();
			G_start_flag=0;
			Save_wall();
			Save_Wall();
			break;
		}
		G_goal_number++;
	}
	wait_ms(500);
}


void Adatimethod_kiti(){
	Load_wall();
	Load_sensor();
	G_start_flag=0;
	G_finish_flag=0;
	G_recostop_flag=0;
	kiti_flag=0;

	mpu6500_do_off();
	G_look_position_y=0;
	G_look_position_x=0;
	G_direction=1080;
	G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
	while(1){
		while(1){
			goal_flag();
			if(G_finish_flag==1){
				G_finish_flag = 0;
				if(G_goalpass_x[G_goal_number]==G_firstgoal_x&&G_goalpass_y[G_goal_number]==G_firstgoal_y){
					move_kabestop();
					Save_wall();
					Save_Wall();
					irumi_LED();
				}
				break;
			}
			pl_hosu_set();
			pl_hosu_make();
			kiti_check();
			if(kiti_flag==1){
				set_goalkiti();
				make_pass_saitan();
			}
			while(1){
				if(kiti_flag==0){
					if((G_V_Motor==0.0)||(G_X_Motor>G_tansakumae)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
						HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
						move_adati2();
						if(G_recostop_flag!=1){
							Record_Wall();
						}
						break;
					}
				}else{
					if((G_V_Motor==0.0)||(G_X_Motor>G_tansakumae)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
						move_kitisaitan2();
						G_look_position_x=G_goalpass_x[G_goal_number];
						G_look_position_y=G_goalpass_y[G_goal_number];
						Record_Wall();
						G_recostop_flag=0;
						kiti_flag=0;
						break;
					}
				}
			}
		}
		if(G_goalpass_x[G_goal_number]==0&&G_goalpass_y[G_goal_number]==0){
			gensoku(G_V_comtur, 0.0, G_A_comst, 90.0, 1, 2);
			move_kabeawase();
			turn_180R();
			G_start_flag=0;
			Save_wall();
			Save_Wall();
			break;
		}
		G_goal_number++;
	}
	wait_ms(500);
}

void Adatimethod_daikiti(){
	int do_dai_flag;
	int start_dai_flag;
	Load_wall();
	Load_sensor();
	Set_Wall();
	G_start_flag=0;
	G_finish_flag=0;
	do_dai_flag=0;
	start_dai_flag=0;
	G_recostop_flag=0;
	kiti_flag=0;

	mpu6500_do_off();
	G_direction=1080;
	G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
	G_look_position_y=0;
	G_look_position_x=0;
	while(1){
		while(1){
			goal_flag();
			if(G_finish_flag==1){
				G_finish_flag = 0;
				if(G_goalpass_x[G_goal_number]==G_firstgoal_x&&G_goalpass_y[G_goal_number]==G_firstgoal_y){
					move_kabestop();
					Save_wall();

					do_dai_flag=1;
					start_dai_flag=1;
					irumi_LED();
				}else if(start_dai_flag==1){
					if(G_goalpass_s[G_goal_number]>0){
						check_daiku();
					}
				}
				break;
			}
			if(do_dai_flag==1){
				do_dai_flag=0;
				daikusutora2();
			}
			pl_hosu_set();
			pl_hosu_make();
			kiti_check();
			if(kiti_flag==1){
				set_goalkiti();
				make_pass_saitan();
			}
			while(1){
				if(kiti_flag==0){
					if((G_V_Motor==0.0)||(G_X_Motor>20.0)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
						move_adati2();
						if(G_recostop_flag!=1){
							Record_Wall();
						}
						break;
					}
				}else{
					if((G_V_Motor==0.0)||(G_X_Motor>20.0)||((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F)){
						move_kitisaitan2();
						G_look_position_x=G_goalpass_x[G_goal_number];
						G_look_position_y=G_goalpass_y[G_goal_number];
						Record_Wall();
						G_recostop_flag=0;
						kiti_flag=0;
						break;
					}
				}
			}
		}
		if(G_goalpass_x[G_goal_number]==0&&G_goalpass_y[G_goal_number]==0){
			gensoku(G_V_comtur, 0.0, G_A_comst, 90.0, 1, 2);
			move_kabeawase();
			turn_180R();
			G_start_flag=0;
			Save_wall();
			break;
		}
		G_goal_number++;
	}
	wait_ms(500);
}
