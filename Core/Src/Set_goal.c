/*
 * Set_goal.c
 *
 *  Created on: Nov 25, 2025
 *      Author: shino
 */

#include "gpio.h"
#include "Set_goal.h"

#include "Wall_record.h"

#include "G_hensu.h"

int i_goal=0;

void reset_goalpass(){
	for(i_goal=0;i_goal<=G_goal_number;i_goal++){
		G_goalpass_x[i_goal]=0;
		G_goalpass_y[i_goal]=0;
		G_goalpass_s[i_goal]=0;
	}
}

void set_goalpass_07(){
	G_goal_x=G_firstgoal_x;
	G_goal_y=G_firstgoal_y;
	G_goalpass_x[0]=G_goal_x;
	G_goalpass_y[0]=G_goal_y;
	G_goalpass_x[1]=0;
	G_goalpass_y[1]=0;
	G_goal_number=0;
}

//void set_goal(){
//	G_goal_x=G_goalpass_x[G_goal_number];
//	G_goal_y=G_goalpass_y[G_goal_number];
//}

void set_goalkiti(){
	int dire_kiti;
	int x_kiti,y_kiti;
	x_kiti=G_look_position_x;
	y_kiti=G_look_position_y;
	dire_kiti=G_direction;
	while(1){
		if (dire_kiti % 360 == 0) {
			if((y_kiti+1<G_maxMAP)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti][y_kiti + 1])
				&&((G_Row[y_kiti+1]&(1<<x_kiti))!=(1<<x_kiti))) {
				y_kiti++;
			}else if ((x_kiti+1<G_maxMAP)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti + 1][y_kiti])
				&&((G_Column[x_kiti+1]&(1<<y_kiti))!=(1<<y_kiti))) {
				x_kiti++;
				dire_kiti+=90;
			}else if((0<=x_kiti-1)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti - 1][y_kiti])
				&&((G_Column[x_kiti]&(1<<y_kiti))!=(1<<y_kiti))) {
				x_kiti--;
				dire_kiti-=90;
			}
		}else if(dire_kiti % 360 == 90){
			if ((x_kiti+1<G_maxMAP)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti+1][y_kiti])
				&&((G_Column[x_kiti+1]&(1<<y_kiti))!=(1<<y_kiti))){
				x_kiti++;
			}else if ((0<=y_kiti-1)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti][y_kiti-1])
				&&((G_Row[y_kiti]&(1<<x_kiti))!=(1<<x_kiti))){
				y_kiti--;
				dire_kiti+=90;
			}else if ((y_kiti+1<G_maxMAP)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti][y_kiti+1])
				&&((G_Row[y_kiti+1]&(1<<x_kiti))!=(1<<x_kiti))){
				y_kiti++;
				dire_kiti-=90;
			}
		}else if(dire_kiti % 360 == 180){
			if ((0<=y_kiti-1)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti][y_kiti-1])
				&&((G_Row[y_kiti]&(1<<x_kiti))!=(1<<x_kiti))) {
				y_kiti--;
			}else if ((0<=x_kiti-1)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti-1][y_kiti])
				&&((G_Column[x_kiti]&(1<<y_kiti))!=(1<<y_kiti))) {
				x_kiti--;
				dire_kiti+=90;
			}else if ((x_kiti+1<G_maxMAP)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti+1][y_kiti])
				&&((G_Column[x_kiti+1]&(1<<y_kiti))!=(1<<y_kiti))) {
				x_kiti++;
				dire_kiti-=90;
			}
		}else{
			if((0<=x_kiti-1)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti - 1][y_kiti])
				&&((G_Column[x_kiti]&(1<<y_kiti))!=(1<<y_kiti))
				&&((G_wall_recoder_flag[y_kiti]&(1<<(x_kiti-1)))==(1<<(x_kiti-1)))) {
				x_kiti--;
			}else if((y_kiti+1<G_maxMAP)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti][y_kiti + 1])
				&&((G_Row[y_kiti+1]&(1<<x_kiti))!=(1<<x_kiti))){
				y_kiti++;
				dire_kiti+=90;
			}else if((0<=y_kiti-1)
				&&(G_dist[x_kiti][y_kiti] > G_dist[x_kiti][y_kiti - 1])
				&&((G_Row[y_kiti]&(1<<x_kiti))!=(1<<x_kiti))){
				y_kiti--;
				dire_kiti-=90;
			}
		}
		if(((G_wall_recoder_flag[y_kiti]&(1<<x_kiti))!=(1<<x_kiti))
			||(G_goalpass_x[G_goal_number]==x_kiti&&G_goalpass_y[G_goal_number]==y_kiti)){
			G_goalpass_x[G_goal_number+1]=G_goalpass_x[G_goal_number];
			G_goalpass_y[G_goal_number+1]=G_goalpass_y[G_goal_number];
			G_goalpass_s[G_goal_number+1]=G_goalpass_s[G_goal_number];
			G_goalpass_x[G_goal_number]=x_kiti;
			G_goalpass_y[G_goal_number]=y_kiti;
			G_goalpass_s[G_goal_number]=0;
			break;
		}
	}
}

//void set_goal_dai(){
//	int i_dai;
//	int x_dai,y_dai;
//
//	i_dai=0;
//	x_dai=G_goal_x;
//	y_dai=G_goal_y;
//	while(1){
//		if(x_dai==0&&y_dai==0){
//			break;
//		}
//		if((y_dai!=15)&&(G_dist[x_dai][y_dai + 1] == G_dist[x_dai][y_dai]+1)){
//			if(G_wall_recoder_flag[x_dai][y_dai+1]==0){
//				G_goalpass_x[G_goal_number+i_dai+2]=G_goalpass_x[G_goal_number+i_dai];
//				G_goalpass_y[G_goal_number+i_dai+2]=G_goalpass_y[G_goal_number+i_dai];
//				G_goalpass_x[G_goal_number+i_dai+1]=x_dai;
//				G_goalpass_y[G_goal_number+i_dai+1]=y_dai+1;
//				G_goalpass_x[G_goal_number+i_dai]=x_dai;
//				G_goalpass_y[G_goal_number+i_dai]=y_dai;
//				if((G_goalpass_x[G_goal_number+i_dai]==G_goalpass_x[G_goal_number+i_dai-1])
//					&&(G_goalpass_y[G_goal_number+i_dai]==G_goalpass_y[G_goal_number+i_dai-1])){
//					G_goalpass_x[G_goal_number+i_dai]=G_goalpass_x[G_goal_number+i_dai+1];
//					G_goalpass_y[G_goal_number+i_dai]=G_goalpass_y[G_goal_number+i_dai+1];
//					G_goalpass_x[G_goal_number+i_dai+1]=G_goalpass_x[G_goal_number+i_dai+2];
//					G_goalpass_y[G_goal_number+i_dai+1]=G_goalpass_y[G_goal_number+i_dai+2];
//				}
//				i_dai++;
//			}
//			y_dai++;
//		}else if ((y_dai!=0)&&(G_dist[x_dai][y_dai-1] == G_dist[x_dai][y_dai]+1)){
//			if(G_wall_recoder_flag[x_dai][y_dai-1]==0){
//				G_goalpass_x[G_goal_number+i_dai+2]=G_goalpass_x[G_goal_number+i_dai];
//				G_goalpass_y[G_goal_number+i_dai+2]=G_goalpass_y[G_goal_number+i_dai];
//				G_goalpass_x[G_goal_number+i_dai+1]=x_dai;
//				G_goalpass_y[G_goal_number+i_dai+1]=y_dai-1;
//				G_goalpass_x[G_goal_number+i_dai]=x_dai;
//				G_goalpass_y[G_goal_number+i_dai]=y_dai;
//				if((G_goalpass_x[G_goal_number+i_dai]==G_goalpass_x[G_goal_number+i_dai-1])
//					&&(G_goalpass_y[G_goal_number+i_dai]==G_goalpass_y[G_goal_number+i_dai-1])){
//					G_goalpass_x[G_goal_number+i_dai]=G_goalpass_x[G_goal_number+i_dai+1];
//					G_goalpass_y[G_goal_number+i_dai]=G_goalpass_y[G_goal_number+i_dai+1];
//					G_goalpass_x[G_goal_number+i_dai+1]=G_goalpass_x[G_goal_number+i_dai+2];
//					G_goalpass_y[G_goal_number+i_dai+1]=G_goalpass_y[G_goal_number+i_dai+2];
//				}
//				i_dai++;
//			}
//			y_dai--;
//		}else if ((x_dai!=15)&&(G_dist[x_dai+1][y_dai] == G_dist[x_dai][y_dai]+1)){
//			if(G_wall_recoder_flag[x_dai+1][y_dai]==0){
//				G_goalpass_x[G_goal_number+i_dai+2]=G_goalpass_x[G_goal_number+i_dai];
//				G_goalpass_y[G_goal_number+i_dai+2]=G_goalpass_y[G_goal_number+i_dai];
//				G_goalpass_x[G_goal_number+i_dai+1]=x_dai+1;
//				G_goalpass_y[G_goal_number+i_dai+1]=y_dai;
//				G_goalpass_x[G_goal_number+i_dai]=x_dai;
//				G_goalpass_y[G_goal_number+i_dai]=y_dai;
//				if((G_goalpass_x[G_goal_number+i_dai]==G_goalpass_x[G_goal_number+i_dai-1])
//					&&(G_goalpass_y[G_goal_number+i_dai]==G_goalpass_y[G_goal_number+i_dai-1])){
//					G_goalpass_x[G_goal_number+i_dai]=G_goalpass_x[G_goal_number+i_dai+1];
//					G_goalpass_y[G_goal_number+i_dai]=G_goalpass_y[G_goal_number+i_dai+1];
//					G_goalpass_x[G_goal_number+i_dai+1]=G_goalpass_x[G_goal_number+i_dai+2];
//					G_goalpass_y[G_goal_number+i_dai+1]=G_goalpass_y[G_goal_number+i_dai+2];
//				}
//				i_dai++;
//			}
//			x_dai++;
//		}else if((x_dai!=0)&&(G_dist[x_dai-1][y_dai] == G_dist[x_dai][y_dai]+1)){
//			if(G_wall_recoder_flag[x_dai-1][y_dai]==0){
//				G_goalpass_x[G_goal_number+i_dai+2]=G_goalpass_x[G_goal_number+i_dai];
//				G_goalpass_y[G_goal_number+i_dai+2]=G_goalpass_y[G_goal_number+i_dai];
//				G_goalpass_x[G_goal_number+i_dai+1]=x_dai-1;
//				G_goalpass_y[G_goal_number+i_dai+1]=y_dai;
//				G_goalpass_x[G_goal_number+i_dai]=x_dai;
//				G_goalpass_y[G_goal_number+i_dai]=y_dai;
//				if((G_goalpass_x[G_goal_number+i_dai]==G_goalpass_x[G_goal_number+i_dai-1])
//					&&(G_goalpass_y[G_goal_number+i_dai]==G_goalpass_y[G_goal_number+i_dai-1])){
//					G_goalpass_x[G_goal_number+i_dai]=G_goalpass_x[G_goal_number+i_dai+1];
//					G_goalpass_y[G_goal_number+i_dai]=G_goalpass_y[G_goal_number+i_dai+1];
//					G_goalpass_x[G_goal_number+i_dai+1]=G_goalpass_x[G_goal_number+i_dai+2];
//					G_goalpass_y[G_goal_number+i_dai+1]=G_goalpass_y[G_goal_number+i_dai+2];
//				}
//				i_dai++;
//			}
//			x_dai--;
//		}
//	}
//}

void set_goal_dai(){
	int x_dai,y_dai;
	int set_goal_flag;

	i_goal=0;
	while(1){
		if(G_goalpass_x[G_goal_number+i_goal]==0&&G_goalpass_y[G_goal_number+i_goal]==0){
			i_goal=0;
			break;
		}else{
			G_goalpass_x[G_goal_number+i_goal]=0;
			G_goalpass_y[G_goal_number+i_goal]=0;
			G_goalpass_s[G_goal_number+i_goal]=0;
		}
		i_goal++;
	}
	x_dai=G_goal_x;
	y_dai=G_goal_y;
	while(1){
		if(x_dai==0&&y_dai==0){
			break;
		}
		if((y_dai+1<G_maxMAP)&&(G_dist[x_dai][y_dai + 1] == G_dist[x_dai][y_dai]+1)){
			if((G_wall_recoder_flag[y_dai+1]&(1<<x_dai))!=(1<<x_dai)){
				if((y_dai+2<G_maxMAP)&&(G_dist[x_dai][y_dai + 2] == G_dist[x_dai][y_dai+1]+1)){
					set_goal_flag=1;
				}else if((x_dai+1<G_maxMAP)&&(G_dist[x_dai+1][y_dai+1] == G_dist[x_dai][y_dai+1]+1)){
					set_goal_flag=3;
				}else if((0<=x_dai-1)&&(G_dist[x_dai-1][y_dai+1] == G_dist[x_dai][y_dai+1]+1)){
					set_goal_flag=4;
				}
			}
			y_dai++;
		}else if ((0<=y_dai-1)&&(G_dist[x_dai][y_dai-1] == G_dist[x_dai][y_dai]+1)){
			if((G_wall_recoder_flag[y_dai-1]&(1<<x_dai))!=(1<<x_dai)){
				if((0<=y_dai-2)&&(G_dist[x_dai][y_dai - 2] == G_dist[x_dai][y_dai-1]+1)){
					set_goal_flag=2;
				}else if((x_dai+1<G_maxMAP)&&(G_dist[x_dai+1][y_dai-1] == G_dist[x_dai][y_dai-1]+1)){
					set_goal_flag=3;
				}else if((0<=x_dai-1)&&(G_dist[x_dai-1][y_dai-1] == G_dist[x_dai][y_dai-1]+1)){
					set_goal_flag=4;
				}
			}
			y_dai--;
		}else if ((x_dai+1<G_maxMAP)&&(G_dist[x_dai+1][y_dai] == G_dist[x_dai][y_dai]+1)){
			if((G_wall_recoder_flag[y_dai]&(1<<(x_dai+1)))!=(1<<(x_dai+1))){
				if((x_dai+2<G_maxMAP)&&(G_dist[x_dai+2][y_dai] == G_dist[x_dai+1][y_dai]+1)){
					set_goal_flag=3;
				}else if((y_dai+1<G_maxMAP)&&(G_dist[x_dai+1][y_dai+1] == G_dist[x_dai+1][y_dai]+1)){
					set_goal_flag=1;
				}else if((0<=y_dai-1)&&(G_dist[x_dai+1][y_dai-1] == G_dist[x_dai+1][y_dai]+1)){
					set_goal_flag=2;
				}
			}
			x_dai++;
		}else if((0<=x_dai-1)&&(G_dist[x_dai-1][y_dai] == G_dist[x_dai][y_dai]+1)){
			if((G_wall_recoder_flag[y_dai]&(1<<(x_dai-1)))!=(1<<(x_dai-1))){
				if((0<=x_dai-2)&&(G_dist[x_dai-2][y_dai] == G_dist[x_dai-1][y_dai]+1)){
					set_goal_flag=4;
				}else if((y_dai+1<G_maxMAP)&&(G_dist[x_dai-1][y_dai+1] == G_dist[x_dai-1][y_dai]+1)){
					set_goal_flag=1;
				}else if((0<=y_dai-1)&&(G_dist[x_dai-1][y_dai-1] == G_dist[x_dai-1][y_dai]+1)){
					set_goal_flag=2;
				}
			}
			x_dai--;
		}
		if(set_goal_flag!=0){
			set_goal_flag=0;
			G_goalpass_x[G_goal_number+i_goal+1]=G_goalpass_x[G_goal_number+i_goal];
			G_goalpass_y[G_goal_number+i_goal+1]=G_goalpass_y[G_goal_number+i_goal];
			G_goalpass_s[G_goal_number+i_goal+1]=G_goalpass_s[G_goal_number+i_goal];
			G_goalpass_x[G_goal_number+i_goal]=x_dai;
			G_goalpass_y[G_goal_number+i_goal]=y_dai;
			if(set_goal_flag==1){
				G_goalpass_s[G_goal_number+i_goal]=1;
			}else if(set_goal_flag==2){
				G_goalpass_s[G_goal_number+i_goal]=2;
			}else if(set_goal_flag==3){
				G_goalpass_s[G_goal_number+i_goal]=3;
			}else if(set_goal_flag==4){
				G_goalpass_s[G_goal_number+i_goal]=4;
			}
			i_goal++;
		}
	}
}

void display_goalpass(){
	int i_debuck;
	for(i_debuck=0; i_debuck<=255; i_debuck++){
		printf("x=%d y=%d s=%d\n\r",G_goalpass_x[i_debuck],G_goalpass_y[i_debuck],G_goalpass_s[i_debuck]);
	}
}
