/*
 * Saitan.c
 *
 *  Created on: Oct 20, 2025
 *      Author: shino
 */

#include "Saitan.h"

#include "stdio.h"
#include "math.h"

#include "PL_sensor.h"		//デバック用
#include "motor_seigyo.h"
#include "move_preset.h"
#include "moveonly_sura.h"
#include "Wall_record.h"
#include "Hosu_make.h"

#include "G_hensu.h"

int x_saitan=0;
int y_saitan=0;
int X_saitan;
int count_saitan;
int count_saitan2;
int count_sinsaitan;
int pass[G_maxMAP*G_maxMAP];
int pass2[G_maxMAP*G_maxMAP][G_maxV];

void trim_map_saitan(){//sintrimがあったら
	for(y_saitan=0;y_saitan<G_maxMAP;y_saitan++){
		for(x_saitan=0;x_saitan<G_maxMAP;x_saitan++){
			G_Row[y_saitan]=G_Row[y_saitan]|(1<<x_saitan);
		}
	}
	for(x_saitan=0;x_saitan<G_maxMAP;x_saitan++){
		for(y_saitan=0;y_saitan<G_maxMAP;y_saitan++){
			G_Column[x_saitan]=G_Column[x_saitan]|(1<<y_saitan);
		}
	}
	for(x_saitan=0;x_saitan<G_maxMAP;x_saitan++){
		for(y_saitan=0;y_saitan<G_maxMAP;y_saitan++){
			if((G_wall_recoder_flag[y_saitan]&(1<<x_saitan))==(1<<x_saitan)){
				if((G_Row_save[y_saitan]&(1<<x_saitan))!=(1<<x_saitan)){
					G_Row[y_saitan]=G_Row[y_saitan]&~(1<<x_saitan);
				}
				if((G_Column_save[x_saitan]&(1<<y_saitan))!=(1<<y_saitan)){
					G_Column[x_saitan]=G_Column[x_saitan]&~(1<<y_saitan);
				}
			}
		}
	}
}

void make_pass_saitan(){
	int pass_dire;

	for(count_saitan=0;count_saitan<=count_saitan2;count_saitan++){
		pass[count_saitan]=0;
	}
	count_saitan=0;
	x_saitan=G_look_position_x;
	y_saitan=G_look_position_y;
	pass_dire=G_direction;
	while(1){
		if (pass_dire % 360 == 0) {
			if (((y_saitan+1)<G_maxMAP)
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan][y_saitan + 1])
				&&((G_Row[y_saitan+1]&(1<<x_saitan))!=(1<<x_saitan))){
				pass[count_saitan] = pass[count_saitan] + 2;
				y_saitan++;
			} else if (((x_saitan+1)<G_maxMAP)
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan + 1][y_saitan])
				&&((G_Column[x_saitan+1]&(1<<y_saitan))!=(1<<y_saitan))) {
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan] = -3;
				count_saitan++;
				x_saitan++;
				pass_dire+=90;
			} else if ((0<=(x_saitan-1))
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan - 1][y_saitan])
				&&((G_Column[x_saitan]&(1<<y_saitan))!=(1<<y_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan] = -2;
				count_saitan++;
				x_saitan--;
				pass_dire-=90;
			}
		}else if(pass_dire % 360 == 90){
			if (((x_saitan+1)<G_maxMAP)
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan+1][y_saitan])
				&&((G_Column[x_saitan+1]&(1<<y_saitan))!=(1<<y_saitan))){
					pass[count_saitan] = pass[count_saitan] + 2;
					x_saitan++;
			} else if ((0<=(y_saitan-1))
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan][y_saitan-1])
				&&((G_Row[y_saitan]&(1<<x_saitan))!=(1<<x_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan] = -3;
				count_saitan++;
				y_saitan--;
				pass_dire+=90;
			} else if (((y_saitan+1)<G_maxMAP)
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan][y_saitan+1])
				&&((G_Row[y_saitan+1]&(1<<x_saitan))!=(1<<x_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan ] = -2;
				count_saitan++;
				y_saitan++;
				pass_dire-=90;
			}
		}else if(pass_dire % 360 == 180){
			if ((0<=(y_saitan-1))
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan][y_saitan-1])
				&&((G_Row[y_saitan]&(1<<x_saitan))!=(1<<x_saitan))){
				pass[count_saitan] = pass[count_saitan] + 2;
				y_saitan--;
			}else if ((0<=(x_saitan-1))
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan-1][y_saitan])
				&&((G_Column[x_saitan]&(1<<y_saitan))!=(1<<y_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan] = -3;
				count_saitan++;
				x_saitan--;
				pass_dire+=90;
			}else if (((x_saitan+1)<G_maxMAP)
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan+1][y_saitan])
				&&((G_Column[x_saitan+1]&(1<<y_saitan))!=(1<<y_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan ] = -2;
				count_saitan++;
				x_saitan++;
				pass_dire-=90;
			}
		}else{
			if((0<=(x_saitan-1))
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan - 1][y_saitan])
				&&((G_Column[x_saitan]&(1<<y_saitan))!=(1<<y_saitan))){
				pass[count_saitan] = pass[count_saitan] + 2;
				x_saitan--;
			}else if(((y_saitan+1)<G_maxMAP)
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan][y_saitan + 1])
				&&((G_Row[y_saitan+1]&(1<<x_saitan))!=(1<<x_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan] = -3;
				count_saitan++;
				y_saitan++;
				pass_dire+=90;
			}else if((0<=(y_saitan-1))
				&&(G_dist[x_saitan][y_saitan] > G_dist[x_saitan][y_saitan - 1])
				&&((G_Row[y_saitan]&(1<<x_saitan))!=(1<<x_saitan))){
				if(pass[count_saitan]>0){
					count_saitan++;
				}
				pass[count_saitan ] = -2;
				count_saitan++;
				y_saitan--;
				pass_dire-=90;
			}
		}
		if(x_saitan==G_goalpass_x[G_goal_number]&&y_saitan==G_goalpass_y[G_goal_number]){
			if(pass[count_saitan]>0){
				count_saitan++;
			}
			count_saitan2=count_saitan;				//pass[count_saitan2]は0最後の印
			break;
		}
	}
//	display_pass();
}

void move_sinsaitan(){
	count_saitan=0;
	while(1){
		if(motor_stop_flag==1){
			break;
		}else if(count_saitan==count_saitan2){
			gensoku_stop(26500);
			DC_motor_stop();
			break;
		}else if(pass[count_saitan]==-2){
			sura_moveL();
		}else if(pass[count_saitan]==-3){
			sura_moveR();
		}else if(pass[count_saitan]==-4){
			daisura_moveL(0);
		}else if(pass[count_saitan]==-5){
			daisura_moveR(0);
		}else if(pass[count_saitan]==-6){
			in180sura_moveL(0);
		}else if(pass[count_saitan]==-7){
			in180sura_moveR(0);
		}else if(pass[count_saitan]==-8){
			in45sura_moveL(0);
		}else if(pass[count_saitan]==-9){
			in45sura_moveR(0);
		}else if(pass[count_saitan]==-10){
			out45sura_moveL(0);
		}else if(pass[count_saitan]==-11){
			out45sura_moveR(0);
		}else if(pass[count_saitan]==-12){
			in135sura_moveL(0);
		}else if(pass[count_saitan]==-13){
			in135sura_moveR(0);
		}else if(pass[count_saitan]==-14){
			out135sura_moveL(0);
		}else if(pass[count_saitan]==-15){
			out135sura_moveR(0);
		}else if(pass[count_saitan]==-16){
			v_90sura_moveL(0);
		}else if(pass[count_saitan]==-17){
			v_90sura_moveR(0);
		}else if(pass[count_saitan]>0){
			if(count_saitan==0){
				X_saitan=(G_kukaku*0.5*pass[count_saitan])-G_kukaku+hazimari;
				if(pass[count_saitan+1]<-3){
					if(pass[count_saitan]==1){
						kasoku(0.0, G_V_maxtur1, G_A_maxst, (hazimari-tohasira), 4, 0);
					}else{
						sakaku_daikei(0.0, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan, 4, 2);
					}
				}else{
					sakaku_daikei(0.0, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 4, 1);
//					if(pass[count_saitan]==2){
//						sakaku_daikei(0.0, G_V_maxst, G_V_comtur, G_A_comst, hazimari, 4, 1);
//					}else if(pass[count_saitan]>=4){
//						sakaku_daikei(0.0, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 4, 1);
//					}
				}
//				X_saitan=(G_kukaku*0.5*pass[count_saitan]-G_kukaku+tohasira);
//				if(X_saitan>0){
//					if(pass[count_saitan+1]<-3){
//						sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan, 0, 0);
//					}else{
//						sakaku_daikei(G_V_comtur, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 0 ,0);
//					}
//				}
			}else if(G_state==1){
				X_saitan=G_kukaku*0.5*pass[count_saitan];
				if((pass[count_saitan-1]<-3)&&(pass[count_saitan+1]<-3)){
					sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan, 2, 2);
				}else if(pass[count_saitan+1]<-3){
					sakaku_daikei(G_V_comtur, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan, 1, 2);
				}else if(pass[count_saitan-1]<-3){
					sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 2, 1);
				}else{
					sakaku_daikei(G_V_comtur, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 1, 1);
				}
			}else if(G_state==4){
				X_saitan=G_kukaku*0.5*pass[count_saitan]*1.4141;
				if((pass[count_saitan-1]<-3)&&(pass[count_saitan+1]<-3)){
					sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan, 3, 3);
//				}else if(pass[count_saitan+1]==0){
//					sakaku_daikei(G_V_comtur, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					G_fix_mode1=0;
//					G_fix_mode2=0;
//					gensoku(G_V_comtur, 0.0, G_A_comst, 90.0);
				}
			}
		}
		count_saitan++;
	}

	for(count_saitan=0;count_saitan<count_saitan2;count_saitan++){
		pass[count_saitan]=0;
	}
	motor_stop_flag=0;
}

void move_sinsaitan2(){
//	count_saitan=0;
//	passcopy();
//	pass_assyuku_sai();
//	while(1){
//		if(motor_stop_flag==1){
//			break;
//		}else if(count_saitan==count_saitan2){
//			move_gensku();
//			break;
//		}else if(pass2[count_saitan][0]==-2){
//			sura_moveL();
//		}else if(pass2[count_saitan][0]==-3){
//			sura_moveR();
//		}else if(pass2[count_saitan][0]==-4){
//			daisura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-5){
//			daisura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-6){
//			in180sura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-7){
//			in180sura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-8){
//			in45sura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-9){
//			in45sura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-10){
//			out45sura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-11){
//			out45sura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-12){
//			in135sura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-13){
//			in135sura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-14){
//			out135sura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-15){
//			out135sura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-16){
//			v_90sura_moveL(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]==-17){
//			v_90sura_moveR(pass2[count_saitan][1]);
//		}else if(pass2[count_saitan][0]>0){
//			if(count_saitan==0){
//				G_fix_mode1=0;
//				G_fix_mode2=0;
//				if(pass2[count_saitan+1][0]<-3){
//					kasoku(0.0, G_V_maxtur1, G_A_maxst, (hazimari-tohasira));
//				}else{
//					kasoku(0.0, G_V_comtur, G_A_maxst, (hazimari-tohasira));
//				}
//				X_saitan=(90.0*pass2[count_saitan][0]-180.0+tohasira);
//				if(X_saitan>0){
//					if(pass2[count_saitan+1][0]<-3){
//						sakaku_daikei(G_V_Motor, G_V_maxst, G_V_maxtur2, G_A_comst, X_saitan);
//					}else{
//						sakaku_daikei(G_V_Motor, G_V_maxst, G_V_comtur, G_A_comst, X_saitan);
//					}
//				}
//			}else if(G_state==1){
//				X_saitan=90.0*pass2[count_saitan][0];
//				if((pass2[count_saitan-1][0]<-3)&&(pass2[count_saitan+1][0]<-3)){
//					G_fix_mode1=2;
//					G_fix_mode2=2;
//					if(pass2[count_saitan][1]==0){
//						sakaku_daikei(G_V_Motor, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==1){
//						sakaku_daikei(G_V_Motor, G_V_maxst, G_V_maxtur2, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==2){
//						sakaku_daikei(G_V_Motor, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==3){
//						sakaku_daikei(G_V_Motor, G_V_maxst, G_V_maxtur2, G_A_comst, X_saitan);
//					}
//				}else if(pass2[count_saitan+1][0]<-3){
//					G_fix_mode1=1;
//					G_fix_mode2=2;
//					sakaku_daikei(G_V_Motor, G_V_maxst, G_V_maxtur2, G_A_comst, X_saitan);
//				}else if(pass2[count_saitan-1][0]<-3){
//					G_fix_mode1=2;
//					G_fix_mode2=1;
//					sakaku_daikei(G_V_Motor, G_V_maxst, G_V_comtur, G_A_comst, X_saitan);
//				}else{
//					G_fix_mode1=1;
//					G_fix_mode2=1;
//					sakaku_daikei(G_V_Motor, G_V_maxst, G_V_comtur, G_A_comst, X_saitan);
//				}
//			}else{
//				X_saitan=90.0*pass2[count_saitan][0]*1.4141;
//				G_fix_mode1=3;
//				if((pass2[count_saitan-1][0]<-3)&&(pass2[count_saitan+1][0]<-3)){
//					if(pass2[count_saitan][1]==0){
//						sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==1){
//						sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==2){
//						sakaku_daikei(G_V_maxtur2, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==3){
//						sakaku_daikei(G_V_maxtur2, G_V_maxst, G_V_maxtur2, G_A_comst, X_saitan);
//					}
//				}else if(count_saitan+1==count_saitan2){
//					if(pass2[count_saitan][1]==0){
//						sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==1){
//						sakaku_daikei(G_V_maxtur1, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==2){
//						sakaku_daikei(G_V_maxtur2, G_V_maxst, G_V_maxtur1, G_A_comst, X_saitan);
//					}else if(pass2[count_saitan][1]==3){
//						sakaku_daikei(G_V_maxtur2, G_V_maxst, G_V_maxtur2, G_A_comst, X_saitan);
//					}
//					gensoku(G_V_Motor, 0.0, G_A_maxst, 90.0, 5, 5);
//				}
//			}
//		}
//		count_saitan++;
//	}
//
//	for(count_saitan=0;count_saitan<count_saitan2;count_saitan++){
//		pass2[count_saitan][0]=0;
//	}
//	motor_stop_flag=0;
}

void passcopy(){
	int i_count;
	i_count=0;
	while(1){
		pass[i_count]=pass2[i_count][0];
		i_count++;
		if(i_count==count_saitan2){
			break;
		}
	}
}

void move_kitisaitan(){	//壁当てあり
//	int i_deback=0;
	count_saitan=0;
	while(1){
		if(motor_stop_flag==1){
			break;
		}else if(count_saitan==count_saitan2){
			break;
		}else if(pass[count_saitan]==-2){
			sura_moveL();
		}else if(pass[count_saitan]==-3){
			sura_moveR();
		}else if(pass[count_saitan]>0){
			X_saitan=G_kukaku*0.5*pass[count_saitan];
			if(count_saitan==0){
				if(G_V_Motor==0){
					if(G_kabeate_flag==1){
						G_kabeate_flag=0;
						X_saitan=X_saitan-(G_kukaku-hazimari);
					}
				}else{
					X_saitan=X_saitan-(G_kukaku/9);
				}
			}
			if(G_V_Motor==0){
				sakaku_daikei(0.0, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 4, 1);
			}else{
				sakaku_daikei(G_V_comtur, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 1, 1);
			}
		}
		count_saitan++;
	}

//	gensoku(G_V_comtur, 0.0, G_A_comst, 70.0);
//	while(1){
//		if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//			for(i_deback=0;i_deback>=count_saitan2;i_deback++){
//				printf("%3d %3d %3d %3d %3d\n\r",pass[i_deback], pass[i_deback+1], pass[i_deback+2], pass[i_deback+3], pass[i_deback+4]);
//				wait_ms(100);
//			}
//		}
//	}
	for(count_saitan=0;count_saitan<count_saitan2;count_saitan++){
		pass[count_saitan]=0;
	}
	motor_stop_flag=0;
}

void move_kitisaitan2(){	//壁当て無し
	count_saitan=0;
	while(1){
		if(motor_stop_flag==1){
			break;
		}else if(count_saitan==count_saitan2){
			break;
		}else if(pass[count_saitan]==-2){
			sura_moveL();
		}else if(pass[count_saitan]==-3){
			sura_moveR();
		}else if(pass[count_saitan]>0){
			X_saitan=G_kukaku*0.5*pass[count_saitan];
			if(count_saitan==0){
				if(G_V_Motor==0){
					if(G_start_flag==1){
						X_saitan=X_saitan-(G_kukaku*0.5);
						sakaku_daikei(0.0, G_V_maxst, G_V_comtur, G_A_comst*0.8, X_saitan, 2, 1);
					}else{
						X_saitan=X_saitan-(G_kukaku-hazimari);
						G_start_flag=1;
						sakaku_daikei(0.0, G_V_maxst, G_V_comtur, G_A_comst*0.8, X_saitan, 4, 1);
					}
				}else{
					X_saitan=X_saitan-G_tansakumae;
					sakaku_daikei(G_V_comtur, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 6, 1);
				}
			}else{
				sakaku_daikei(G_V_comtur, G_V_maxst, G_V_comtur, G_A_comst, X_saitan, 1, 1);
			}
		}
		count_saitan++;
	}
	for(count_saitan=0;count_saitan<count_saitan2;count_saitan++){
		pass[count_saitan]=0;
	}
	motor_stop_flag=0;
}

//void pass_assyuku(){
//	int cnt_asyu, cnt_asyu2;
//	int sub_pass[300];
//
//	cnt_asyu2=0;
//	for(count_saitan=0;count_saitan<=count_saitan2;count_saitan++){
//		sub_pass[count_saitan]=pass[count_saitan];
//	}
//
//	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]>=0)){
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-4;				//左dai90
//			pass[cnt_asyu+1]--;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]>=0)){
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-5;				//右dai90
//			pass[cnt_asyu+1]--;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]>=0)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-6;				//左180
//			pass[cnt_asyu+1]--;
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]>=0)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-7;				//右180
//			pass[cnt_asyu+1]--;
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//	}
//}

void pass_assyuku(){
	//pass　偶数は左，奇数は右まわり
	int cnt_asyu;
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-4;				//左90
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-5;				//右90
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]==-2)
			&&(pass[cnt_asyu+2]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-6;				//左180
			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]==-3)
			&&(pass[cnt_asyu+2]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-7;				//右180
			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
	}
}

void pass_assyuku_na(){
	//pass　偶数は左，奇数は右まわり
	int cnt_asyu, cnt_asyu2;
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-4;				//左90
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-5;				//右90
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]==-2)
			&&(pass[cnt_asyu+2]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-6;				//左180
			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]==-3)
			&&(pass[cnt_asyu+2]>=0)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-7;				//右180
			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
			if(pass[cnt_asyu+1]>0){
				pass[cnt_asyu+1]--;
			}
		}
	}
	display_pass();
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if((pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]>0)){
			pass[cnt_asyu+1]--;
			if(pass[cnt_asyu-1]==-3){
				pass[cnt_asyu]=-10;			//左out45
			}else if(pass[cnt_asyu-1]==-2){
				pass[cnt_asyu-1]=-14;		//左out135
				for(count_saitan=cnt_asyu;count_saitan<=count_saitan2;count_saitan++){
					pass[count_saitan]=pass[count_saitan+1];
				}
				count_saitan2--;
			}
		}
		if((pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]>0)){
			pass[cnt_asyu+1]--;
			if(pass[cnt_asyu-1]==-2){
				pass[cnt_asyu]=-11;			//右out45
			}else if(pass[cnt_asyu-1]==-3){
				pass[cnt_asyu-1]=-15;		//右out135
				for(count_saitan=cnt_asyu;count_saitan<=count_saitan2;count_saitan++){
					pass[count_saitan]=pass[count_saitan+1];
				}
				count_saitan2--;
			}
		}
	}
	display_pass();
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]==-2)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-12;			//左in135
			for(count_saitan=cnt_asyu+1;count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]==-3)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-13;			//右in135
			for(count_saitan=cnt_asyu+1;count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
		}
	}
	display_pass();
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-2)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-8;			//左in45
		}
		if((pass[cnt_asyu-1]>0)
			&&(pass[cnt_asyu]==-3)){
			pass[cnt_asyu-1]--;
			pass[cnt_asyu]=-9;			//右in45
		}
	}
	display_pass();
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if((pass[cnt_asyu]==-2)
			&&(pass[cnt_asyu+1]==-2)){
			pass[cnt_asyu]=-16;			//左v90
			for(count_saitan=cnt_asyu+1;count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
		}
		if((pass[cnt_asyu]==-3)
			&&(pass[cnt_asyu+1]==-3)){
			pass[cnt_asyu]=-17;			//右v90
			for(count_saitan=cnt_asyu+1;count_saitan<=count_saitan2;count_saitan++){
				pass[count_saitan]=pass[count_saitan+1];
			}
			count_saitan2--;
		}
	}
	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
		if(pass[cnt_asyu]==-2||pass[cnt_asyu]==-3){
			pass[cnt_asyu]=1;
			cnt_asyu2=0;
			while(1){
				if(pass[cnt_asyu+cnt_asyu2+1]==-2||pass[cnt_asyu+cnt_asyu2+1]==-3){
					pass[cnt_asyu]++;
					cnt_asyu2++;
				}else{
					if(cnt_asyu2>0){
						for(count_saitan=cnt_asyu+1;count_saitan<=count_saitan2;count_saitan++){
							pass[count_saitan]=pass[count_saitan+cnt_asyu2];
						}
						count_saitan2=count_saitan2-cnt_asyu2;
					}
					break;
				}
			}
		}
	}
//	printf("const_saitan2(最後のパスの1つ後の0)=%d\n\r",count_saitan2);
}

//void pass_assyuku_na(){
////pass　偶数は左，奇数は右まわり
//	int na_st_flag=0;
//	int cnt_asyu, cnt_asyu2;
//	int sub_pass[300];
//
//	na_st_flag=0;
//	cnt_asyu2=0;
//	for(count_saitan=0;count_saitan<=count_saitan2;count_saitan++){
//		sub_pass[count_saitan]=pass[count_saitan];
//	}
//
//	for(cnt_asyu=1;cnt_asyu<=count_saitan2;cnt_asyu++){
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]>=0)){
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-4;				//左dai90
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]>=0)){
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-5;				//右dai90
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]>=0)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-6;				//左180
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]>=0)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-7;				//右180
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-3)){
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-8;				//左in45
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-2)){
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-9;				//右in45
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]>=0)){
//			pass[cnt_asyu]=-10;				//左out45
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]>=0)){
//			pass[cnt_asyu]=-11;				//右out45
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]==-3)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-12;				//左in135
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]>0)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]==-2)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu-1]--;
//			pass[cnt_asyu]=-13;				//右in135
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]>=0)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu]=-14;				//左out135
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]>=0)){
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			pass[cnt_asyu]=-15;				//右out135
//			if(sub_pass[cnt_asyu+cnt_asyu2+1]!=0){
//				pass[cnt_asyu+1]--;
//			}
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]==-3)){
//			pass[cnt_asyu]=-16;				//左v90
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//		if((sub_pass[cnt_asyu+cnt_asyu2-1]==-2)
//			&&(sub_pass[cnt_asyu+cnt_asyu2]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+1]==-3)
//			&&(sub_pass[cnt_asyu+cnt_asyu2+2]==-2)){
//			pass[cnt_asyu]=-17;				//右v90
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+1];
//			}
//			count_saitan2--;
//			cnt_asyu2++;
//		}
//	}
//	for(cnt_asyu=0;cnt_asyu<=count_saitan2;cnt_asyu++){
//		if(pass[cnt_asyu]==-2||pass[cnt_asyu]==-3){
//			pass[cnt_asyu]=1;
//			na_st_flag=1;
//		}
//
//		if(na_st_flag==1){
//			cnt_asyu2=0;
//			while(1){
//				if(pass[cnt_asyu+cnt_asyu2+1]==-3||pass[cnt_asyu+cnt_asyu2+1]==-2){
//					pass[cnt_asyu]+=1;
//					cnt_asyu2++;
//				}else{
//					na_st_flag=0;
//					break;
//				}
//			}
//
//			for(count_saitan=(cnt_asyu+1);count_saitan<=count_saitan2;count_saitan++){
//				pass[count_saitan]=pass[count_saitan+cnt_asyu2];
//			}
//			count_saitan2=count_saitan2-cnt_asyu2;
//		}
//	}
//	printf("const_saitan2(最後のパスの1つ後0), %d\n\r",count_saitan2);
//}

void pass_assyuku_sai(){
	int i_saitan;
	i_saitan=0;
	while(1){
		if(pass2[i_saitan][1]==0){
			if(pass2[i_saitan+1][0]>0){
				pass2[i_saitan][1]=1;
				pass2[i_saitan+1][1]=3;
			}
			if((-8<pass2[i_saitan+1][0])&&(pass2[i_saitan+1][0]<-3)){
				pass2[i_saitan][1]=1;
				pass2[i_saitan+1][1]=3;
			}
		}else if(pass2[i_saitan][1]==1){
			if(pass2[i_saitan+1][0]<-7||pass2[i_saitan+1][0]>-4){
				pass2[i_saitan][1]=0;
				pass2[i_saitan+1][1]=1;
			}
		}else if(pass2[i_saitan][1]==3){
			if(pass2[i_saitan+1][0]<-7||pass2[i_saitan+1][0]>-4){
				pass2[i_saitan][1]=2;
				pass2[i_saitan+1][1]=1;
			}
		}
		i_saitan++;
		if(i_saitan==count_saitan2){
			break;
		}
	}
}

void display_pass(){
	int i_dispass;
	for(i_dispass=0;i_dispass<=count_saitan2;i_dispass++){
		printf("%4d,",pass[i_dispass]);
		if(i_dispass%5==4){
			printf("\n\r");
		}
	}
	printf("\n\r");
	printf("finish_x=%d finish_y=%d\n\r",x_saitan,y_saitan);
}

