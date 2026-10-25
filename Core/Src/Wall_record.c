/*
 * Wall_record.c
 *
 *  Created on: Oct 7, 2025
 *      Author: shino
 */

#include "Wall_record.h"

#include "stdio.h"
#include "math.h"
#include "usart.h"
#include "Wall_Control.h"

#include "PL_sensor.h"
#include "G_hensu.h"

int G_wall_recoder_flag[G_maxMAP];
int G_finish_flag=0;

int G_look_position_x=0;
int G_look_position_y=0;
int G_direction=1080;

int G_firstgoal_x=7;
int G_firstgoal_y=7;
int G_goal_x=0;
int G_goal_y=0;
int G_goal_z=0;
int G_goal_s=0;		//0左下，1右下,2左上,3右上
int G_goalpass_x[G_maxMAP*G_maxMAP];
int G_goalpass_y[G_maxMAP*G_maxMAP];
int G_goalpass_s[G_maxMAP*G_maxMAP];
int G_goal_number;

int G_Column[G_maxMAP];
int G_Row[G_maxMAP];
int G_Column_save[G_maxMAP];
int G_Row_save[G_maxMAP];
int G_dist[G_maxMAP][G_maxMAP];

void Set_Wall(){
	G_Row[0] = 0b1111111111111111;
	G_Column[0] = 0b1111111111111111;
	G_Column[1] = 0b0000000000000001;
}

void Set_Wall2(){
    int x_saitan, y_saitan;
	G_Row[0] = 0b1111111111111111;
	G_Row[1] = 0b1010110011110100;
	G_Row[2] = 0b0100110010101010;
	G_Row[3] = 0b1001110011010000;
	G_Row[4] = 0b0111100010101000;
	G_Row[5] = 0b0111110001010000;
	G_Row[6] = 0b0100001010101000;
	G_Row[7] = 0b0100000100010000;
	G_Row[8] = 0b0110001001100000;
	G_Row[9] = 0b0011010110110000;
	G_Row[10] = 0b0010111001110000;
    G_Row[11] = 0b0101010010100000;
    G_Row[12] = 0b0011000001011000;
    G_Row[13] = 0b0010110111010000;
    G_Row[14] = 0b0101110001110000;
    G_Row[15] = 0b0010101111111000;
    G_Column[0] = 0b1111111111111111;
    G_Column[1] = 0b0111111111111101;
    G_Column[2] = 0b1111111111011110;
    G_Column[3] = 0b0111101110101010;
    G_Column[4] = 0b0010110011010100;
    G_Column[5] = 0b0000000010101010;
    G_Column[6] = 0b0000000101010100;
    G_Column[7] = 0b0010001110101000;
    G_Column[8] = 0b0100101001011010;
    G_Column[9] = 0b0101110010111110;
	G_Column[10] = 0b0000100101011010;
    G_Column[11] = 0b1001001011000000;
    G_Column[12] = 0b0000010111000010;
	G_Column[13] = 0b0000101011000101;
    G_Column[14] = 0b0011011010101010;
    G_Column[15] = 0b0110101010100001;
    for(x_saitan=0;x_saitan<G_maxMAP;x_saitan++){
        for(y_saitan=0;y_saitan<G_maxMAP;y_saitan++){
        	G_wall_recoder_flag[y_saitan]=G_wall_recoder_flag[G_look_position_y]|(1<<G_look_position_x);
        }
    }
}

void Set_Wall3(){
	G_Row[0] = 0b1111111111111111;
	G_Row[9] = 0b0000000000000111;
    G_Row[16] = 0b1111111111111111;
    G_Column[0] = 0b1111111111111111;
    G_Column[1] = 0b0000000010111111;
    G_Column[4] = 0b0000001111000000;
    G_Column[16] = 0b1111111111111111;
    G_wall_recoder_flag[0]=G_wall_recoder_flag[0]|(1<<0);
    G_wall_recoder_flag[1]=G_wall_recoder_flag[1]|(1<<0);
    G_wall_recoder_flag[2]=G_wall_recoder_flag[2]|(1<<0);
    G_wall_recoder_flag[3]=G_wall_recoder_flag[3]|(1<<0);
    G_wall_recoder_flag[4]=G_wall_recoder_flag[4]|(1<<0);
    G_wall_recoder_flag[5]=G_wall_recoder_flag[5]|(1<<0);
    G_wall_recoder_flag[6]=G_wall_recoder_flag[6]|(1<<0);
}

void Record_Wall(){
	if(G_direction%360==0){
		if(G_kabeF_flag==1){
			if(G_look_position_y+1<G_maxMAP){
				G_Row[G_look_position_y+1]=G_Row[G_look_position_y+1]|(1<<G_look_position_x);
			}
		}
		if(G_kabeR_flag==1){
			if(G_look_position_x+1<G_maxMAP){
				G_Column[G_look_position_x+1]=G_Column[G_look_position_x+1]|(1<<G_look_position_y);
			}
		}
		if(G_kabeL_flag==1){
			G_Column[G_look_position_x]=G_Column[G_look_position_x]|(1<<G_look_position_y);
		}
	}else if(G_direction%360==90){
		if(G_kabeF_flag==1){
			if(G_look_position_x+1<G_maxMAP){
				G_Column[G_look_position_x+1]=G_Column[G_look_position_x+1]|(1<<G_look_position_y);
			}
		}
		if(G_kabeR_flag==1){
			G_Row[G_look_position_y]=G_Row[G_look_position_y]|(1<<G_look_position_x);
		}
		if(G_kabeL_flag==1){
			if(G_look_position_y+1<G_maxMAP){
				G_Row[G_look_position_y+1]=G_Row[G_look_position_y+1]|(1<<G_look_position_x);
			}
		}
	}else if(G_direction%360==180){
		if(G_kabeF_flag==1){
			G_Row[G_look_position_y]=G_Row[G_look_position_y]|(1<<G_look_position_x);
		}
		if(G_kabeR_flag==1){
			G_Column[G_look_position_x]=G_Column[G_look_position_x]|(1<<G_look_position_y);
		}
		if(G_kabeL_flag==1){
			if(G_look_position_x+1<G_maxMAP){
				G_Column[G_look_position_x+1]=G_Column[G_look_position_x+1]|(1<<G_look_position_y);
			}
		}
	}else{
		if(G_kabeF_flag==1){
			G_Column[G_look_position_x]=G_Column[G_look_position_x]|(1<<G_look_position_y);
		}
		if(G_kabeR_flag==1){
			if(G_look_position_y+1<G_maxMAP){
				G_Row[G_look_position_y+1]=G_Row[G_look_position_y+1]|(1<<G_look_position_x);
			}
		}
		if(G_kabeL_flag==1){
			G_Row[G_look_position_y]=G_Row[G_look_position_y]|(1<<G_look_position_x);
		}
	}
	G_wall_recoder_flag[G_look_position_y]=G_wall_recoder_flag[G_look_position_y]|(1<<G_look_position_x);
}
void Save_Wall(){
	int xy_savewall;
	for(xy_savewall=0;xy_savewall<G_maxMAP;xy_savewall++){
		G_Row_save[xy_savewall]=G_Row[xy_savewall];
		G_Column_save[xy_savewall]=G_Column[xy_savewall];
	}
}

void Load_Wall(){
	int i_loadwall;
	for(i_loadwall=0;i_loadwall<G_maxMAP;i_loadwall++){
		G_Row[i_loadwall]=G_Row_save[i_loadwall];
		G_Column[i_loadwall]=G_Column_save[i_loadwall];
	}
}

void display_wallhosu(){
	int sub_x_position;
	int sub_y_position;
	printf("+");
	for(sub_x_position=0;sub_x_position<G_maxMAP;sub_x_position++){
		printf("----+");
	}
	printf("\r\n");
	for(sub_y_position=G_maxMAP-1;sub_y_position>=0;sub_y_position--){

		for(sub_x_position=0;sub_x_position<G_maxMAP;sub_x_position++){
			if((G_Column[sub_x_position]&(1<<sub_y_position))==(1<<sub_y_position)){
				printf("|%4d",G_dist[sub_x_position][sub_y_position]);
			}else{
				printf(" %4d",G_dist[sub_x_position][sub_y_position]);
			}
		}
		printf("|");
		printf("\r\n");

		for(sub_x_position=0;sub_x_position<G_maxMAP;sub_x_position++){
			if((G_Row[sub_y_position]&(1<<sub_x_position))==(1<<sub_x_position)){
				printf("+----");
			}else{
				printf("     ");
			}
		}
		printf("+");
		printf("\r\n");
	}
	printf("%d  %d",G_direction,G_direction%360);
	printf("\r\n");
	printf("%d  %d ",G_look_position_x,G_look_position_y);
	printf("\r\n");
	printf("%d  %d  %d",G_goal_x,G_goal_y,G_goal_s);
	printf("\r\n");
}

void display_recowall(){
	int sub_x_position;
	int sub_y_position;
	printf("+");
	for(sub_x_position=0;sub_x_position<G_maxMAP;sub_x_position++){
		printf("----+");
	}
	printf("\r\n");
	for(sub_y_position=G_maxMAP-1;sub_y_position>=0;sub_y_position--){

		for(sub_x_position=0;sub_x_position<G_maxMAP;sub_x_position++){
			if((G_Column[sub_x_position]&(1<<sub_y_position))==(1<<sub_y_position)){
				if((G_wall_recoder_flag[sub_y_position]&(1<<sub_x_position))==(1<<sub_x_position)){
					printf("|  1 ");
				}else{
					printf("|  0 ");
				}
			}else{
				if((G_wall_recoder_flag[sub_y_position]&(1<<sub_x_position))==(1<<sub_x_position)){
					printf("   1 ");
				}else{
					printf("   0 ");
				}
			}
		}
		printf("|");
		printf("\r\n");

		for(sub_x_position=0;sub_x_position<G_maxMAP;sub_x_position++){
			if((G_Row[sub_y_position]&(1<<sub_x_position))==(1<<sub_x_position)){
				printf("+----");
			}else{
				printf("     ");
			}
		}
		printf("+");
		printf("\r\n");
	}
	printf("%d  %d",G_direction,G_direction%360);
	printf("\r\n");
	printf("%d  %d ",G_look_position_x,G_look_position_y);
	printf("\r\n");
	printf("%d  %d  %d",G_goal_x,G_goal_y,G_goal_s);
	printf("\r\n");
}
