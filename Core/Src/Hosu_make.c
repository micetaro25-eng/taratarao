/*
 * Hosu_make.c
 *
 *  Created on: Oct 7, 2025
 *      Author: shino
 */

#include "gpio.h"
#include "stdio.h"
#include "math.h"

#include "PL_timer.h"

#include "Hosu_make.h"
#include "Wall_record.h"

#include "G_hensu.h"

int8_t i_hosu=0;
int8_t j_hosu=0;
int8_t k_hosu=0;
int8_t q_x[(G_maxMAP*G_maxMAP)+((G_maxMAP*2)*G_maxV)+(2*G_maxV)+1];					//キュー256+直線96+ターン6+配列分1
int8_t q_y[(G_maxMAP*G_maxMAP)+((G_maxMAP*2)*G_maxV)+(2*G_maxV)+1];
int8_t q_z[(G_maxMAP*G_maxMAP)+((G_maxMAP*2)*G_maxV)+(2*G_maxV)+1];
int8_t q_kabe[(G_maxMAP*G_maxMAP)+((G_maxMAP*2)*G_maxV)+(2*G_maxV)+1];
int16_t head=0;
int16_t tail=0;

int distgoal_mini;
int8_t x_goal_dai, y_goal_dai;
int8_t x_daiku, y_daiku, z_daiku, kabe_daiku;							//実際のゴールの壁が入る

uint16_t omomi_ST[(G_maxMAP*2)][G_maxV][G_maxV];			//直線の重み[0]が0の直進時間
uint16_t omomi_naST[(G_maxMAP*2)][G_maxV][G_maxV];			//直線の重み[0]が0の(90*1.4141)での直進時間

int omomi_TR90[G_maxV][G_maxV];								//前距離，後距離，ターンの時間			0
int omomi_TR180[G_maxV][G_maxV];							//前距離，後距離，ターンの時間			1
int omomi_TR45in[G_maxV][G_maxV];							//前距離，後距離，ターンの時間			2
int omomi_TR45out[G_maxV][G_maxV];							//前距離，後距離，ターンの時間			3
int omomi_TR135in[G_maxV][G_maxV];							//前距離，後距離，ターンの時間			4
int omomi_TR135out[G_maxV][G_maxV];							//前距離，後距離，ターンの時間			5
int omomi_TRv90[G_maxV][G_maxV];							//前距離，後距離，ターンの時間			6

uint16_t omomi_startTR45[G_maxMAP][G_maxV];					//ターン45入りまで
uint16_t omomi_startTR90[G_maxV];							//ターン90最初の奴
uint16_t omomi_startTR135[G_maxV];							//ターン135最初の奴

typedef struct daiku{
	int dist;
	int8_t dire;
	int16_t st;	//加速情報や，ターン直後の状態，-1はターンして直進に入った後，-2は45inターンした後，-3は135inターンした後，-4はv90ターンしたあと
}daiku;
daiku daiRow[G_maxMAP][G_maxMAP][G_maxV];
daiku daiCol[G_maxMAP][G_maxMAP][G_maxV];

void pl_hosu_set(){
	for(i_hosu=0;i_hosu<G_maxMAP;i_hosu++){
		for(j_hosu=0;j_hosu<G_maxMAP;j_hosu++){
			G_dist[i_hosu][j_hosu]=255;
		}
	}
}

void pl_hosu_make(){ //9割完成
//初期化
	head=0;
	tail=0;
//
//初期値
	q_x[tail]=G_goalpass_x[G_goal_number];
	q_y[tail]=G_goalpass_y[G_goal_number];
	G_dist[q_x[tail]][q_y[tail]]=0;
	tail++;
//
	while(1){
		if((q_x[head]+1)<G_maxMAP){
			if ((G_dist[q_x[head]+1][q_y[head]]>G_dist[q_x[head]][q_y[head]]+1)
				&&((G_Column[q_x[head]+1]&(1<<q_y[head]))!=(1<<q_y[head]))){
				q_x[tail] = q_x[head]+1;
				q_y[tail] = q_y[head];
				G_dist[q_x[tail]][q_y[tail]]=G_dist[q_x[head]][q_y[head]]+1;
				tail++;
			}
		}
		if(0<=(q_x[head]-1)){
			if ((G_dist[q_x[head]-1][q_y[head]]>G_dist[q_x[head]][q_y[head]]+1)
				&&((G_Column[q_x[head]]&(1<<q_y[head]))!=(1<<q_y[head]))){
				q_x[tail] = q_x[head]-1;
				q_y[tail] = q_y[head];
				G_dist[q_x[tail]][q_y[tail]]=G_dist[q_x[head]][q_y[head]]+1;
				tail++;
			}
		}
		if((q_y[head]+1)<G_maxMAP){
			if ((G_dist[q_x[head]][q_y[head]+1]>G_dist[q_x[head]][q_y[head]]+1)
				&&((G_Row[q_y[head]+1]&(1<<q_x[head]))!=(1<<q_x[head]))){
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head]+1;
				G_dist[q_x[tail]][q_y[tail]]=G_dist[q_x[head]][q_y[head]]+1;
				tail++;
			}
		}
		if(0<=(q_y[head]-1)){
			if ((G_dist[q_x[head]][q_y[head]-1]>G_dist[q_x[head]][q_y[head]]+1)
				&&((G_Row[q_y[head]]&(1<<q_x[head]))!=(1<<q_x[head]))){
				q_x[tail] = q_x[head];
				q_y[tail] = q_y[head]-1;
				G_dist[q_x[tail]][q_y[tail]]=G_dist[q_x[head]][q_y[head]]+1;
				tail++;
			}
		}
		head++;
		if(head==tail){
			break;
		}
		if(tail>(G_maxMAP*G_maxMAP)){
			for(i_hosu=0;i_hosu<=(tail-head);i_hosu++){
				q_x[i_hosu]=q_x[i_hosu+head];
				q_y[i_hosu]=q_y[i_hosu+head];
			}
			tail=(tail-head)+1;
			head=0;
		}
	}
}

void omomi_set(int mode){
	if(G_maxV==1){
		omomi_TR90[0][0]=omomi11_90;
		omomi_TR180[0][0]=omomi11_180;
		omomi_TR45in[0][0]=omomi11_45in;
		omomi_TR45out[0][0]=omomi11_45out;
		omomi_TR135in[0][0]=omomi11_135in;
		omomi_TR135out[0][0]=omomi11_135out;
		omomi_TRv90[0][0]=omomi11_v90;
	}else if(G_maxV==2){
		omomi_TR90[0][0]=omomi11_90;
		omomi_TR180[0][0]=omomi11_180;
		omomi_TR45in[0][0]=omomi11_45in;
		omomi_TR45out[0][0]=omomi11_45out;
		omomi_TR135in[0][0]=omomi11_135in;
		omomi_TR135out[0][0]=omomi11_135out;
		omomi_TRv90[0][0]=omomi11_v90;
		omomi_TR90[0][1]=omomi12_90;
		omomi_TR180[0][1]=omomi12_180;
		omomi_TR45in[0][1]=omomi12_45in;
		omomi_TR45out[0][1]=omomi12_45out;
		omomi_TR135in[0][1]=omomi12_135in;
		omomi_TR135out[0][1]=omomi12_135out;
		omomi_TRv90[0][1]=omomi12_v90;
		omomi_TR90[1][0]=omomi21_90;
		omomi_TR180[1][0]=omomi21_180;
		omomi_TR45in[1][0]=omomi21_45in;
		omomi_TR45out[1][0]=omomi21_45out;
		omomi_TR135in[1][0]=omomi21_135in;
		omomi_TR135out[1][0]=omomi21_135out;
		omomi_TRv90[1][0]=omomi21_v90;
		omomi_TR90[1][1]=omomi22_90;
		omomi_TR180[1][1]=omomi22_180;
		omomi_TR45in[1][1]=omomi22_45in;
		omomi_TR45out[1][1]=omomi22_45out;
		omomi_TR135in[1][1]=omomi22_135in;
		omomi_TR135out[1][1]=omomi22_135out;
		omomi_TRv90[1][1]=omomi22_v90;
	}
//	setT_for_omomi(mode);
//	omomi_ST[0][]=0;
//	omomi_ST[1][]=((int)(G_omomi_kasoku*1000));
//	for(i_hosu=2;i_hosu<(G_maxMAP*2);i_hosu++){
//		omomi_ST11[i_hosu]=((int)((G_STt1[i_hosu]+G_STt2[i_hosu]+G_STt3[i_hosu])*1000));
//	}
//
//	omomi_naST11[0]=0;
//	for(i_hosu=1;i_hosu<(G_maxMAP*2);i_hosu++){
//		omomi_naST11[i_hosu]=((int)((G_naSTt1[i_hosu]+G_naSTt2[i_hosu]+G_naSTt3[i_hosu])*1000));
//	}
//
//	omomi_startTR45[0][0]=0;
//	omomi_startTR45[1][0]=((int)((G_start_T1+G_start_T3+G_TRt1[2]+G_TRt2[2]+G_TRt3[2]+((G_mae_xt[2]+G_ato_xt[2])/G_zyusin_vt[2]))*1000));
//	for(i_hosu=2;i_hosu<G_maxMAP;i_hosu++){
//		omomi_startTR4511[i_hosu]=((int)((G_start_T1+G_start_T3+G_startSTt1[i_hosu-1]+G_startSTt2[i_hosu-1]+G_startSTt3[i_hosu-1])*1000))+omomi_TR45in11;
//	}
//	omomi_startTR90=((int)((G_start_T1+G_start_T3+G_TRt1[0]+G_TRt2[0]+G_TRt3[0]+((G_mae_xt[0]+G_ato_xt[0])/G_zyusin_vt[0]))*1000));
//	omomi_startTR135=((int)((G_start_T1+G_start_T3+G_TRt1[4]+G_TRt2[4]+G_TRt3[4]+((G_mae_xt[4]+G_ato_xt[4])/G_zyusin_vt[4]))*1000));

//	for(i_hosu=0;i_hosu<=32;i_hosu++){
//		printf("%d ", omomi_ST11[i_hosu]);
//	}
//	printf("\r\n");
//	for(i_hosu=0;i_hosu<=32;i_hosu++){
//		printf("%d ", omomi_naST11[i_hosu]);
//	}
//	printf("\r\n");
//	for(i_hosu=0;i_hosu<=15;i_hosu++){
//		printf("%d ", omomi_startTR4511[i_hosu]);
//	}
//	printf("\r\n");
//	printf("omomi90  180  %d  %d ",omomi_TR9011,omomi_TR18011);
//	printf("\r\n");
//	printf("omomiv90  %d ",omomi_TRv9011);
//	printf("\r\n");
//	printf("omomi45  %d  %d ",omomi_TR45in11,omomi_TR45out11);
//	printf("\r\n");
//	printf("omomi135  %d  %d ",omomi_TR135in11,omomi_TR135out11);
//	printf("\r\n");
//	printf("start  %d  %d ",omomi_startTR90,omomi_startTR135);
//	printf("\r\n");
}

void daiku_hosumake(int mode){
//初期化
	omomi_set(mode);
	head=0;
	tail=0;

	for(i_hosu=0;i_hosu<G_maxMAP;i_hosu++){
		for(j_hosu=0;j_hosu<G_maxMAP;j_hosu++){
			for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
				daiRow[i_hosu][j_hosu][k_hosu].dist=99999;
				daiCol[i_hosu][j_hosu][k_hosu].dist=99999;
			}
		}
	}

	for(i_hosu=0;i_hosu<G_maxMAP;i_hosu++){
		for(j_hosu=0;j_hosu<G_maxMAP;j_hosu++){
			for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
				daiRow[i_hosu][j_hosu][k_hosu].dire=100;
				daiCol[i_hosu][j_hosu][k_hosu].dire=100;
			}
		}
	}

	for(i_hosu=0;i_hosu<G_maxMAP;i_hosu++){
		for(j_hosu=0;j_hosu<G_maxMAP;j_hosu++){
			for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
				daiRow[i_hosu][j_hosu][k_hosu].st=0;
				daiCol[i_hosu][j_hosu][k_hosu].st=0;
			}
		}
	}

	for(i_hosu=0; i_hosu<(G_maxMAP*G_maxMAP)+((G_maxMAP*2)*G_maxV)+(2*G_maxV)+1; i_hosu++){
		q_x[i_hosu] = 0;
		q_y[i_hosu] = 0;
		q_z[i_hosu] = 0;
		q_kabe[i_hosu]=	0;
	}
//
//初期値

	for(i_hosu=0;i_hosu<G_maxV;i_hosu++){
		for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
			j_hosu=0;
			while(1){
				if(((1+j_hosu)<G_maxMAP)
					&&((G_Row[1+j_hosu]&(1<<0))!=(1<<0))){
					if(j_hosu==0){
						daiRow[0][1][0].dire=0;
						daiRow[0][1][0].st=1;
					}else{
						daiRow[0][1+j_hosu][i_hosu].dire=0;
						daiRow[0][1+j_hosu][i_hosu].st=1+2*(j_hosu)+200*i_hosu;
					}
					if((G_Column[1]&(1<<(1+j_hosu)))!=(1<<(1+j_hosu))){
						if(j_hosu==0){
							daiCol[1][1][0].dist=omomi_startTR45[1][k_hosu];
							daiCol[1][1][0].dire=1;
							daiCol[1][1][0].st=-10;
						}else{
							daiCol[1][1+j_hosu][k_hosu].dist=omomi_startTR45[1+j_hosu][k_hosu];
							daiCol[1][1+j_hosu][k_hosu].dire=1;
							daiCol[1][1+j_hosu][k_hosu].st=-2-10*(3*i_hosu+k_hosu);
						}
						q_x[tail] = 1;
						q_y[tail] = 1+j_hosu;
						q_z[tail] = k_hosu;
						q_kabe[tail]=1;
						tail++;
					}
					j_hosu++;
				}else{
					break;
				}
			}
		}
	}

	while(1){
		if((q_kabe[head]==0)&&(0<q_y[head])){
			if(daiRow[q_x[head]][q_y[head]][q_z[head]].dire==0){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					j_hosu=0;
					while(1){
						if(((q_y[head]+1+j_hosu)<G_maxMAP)&&((G_Row[q_y[head]+1+j_hosu]&(1<<q_x[head]))!=(1<<q_x[head]))){
							if(daiRow[q_x[head]][q_y[head]+1+j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*j_hosu)][q_z[head]][k_hosu])){
								daiRow[q_x[head]][q_y[head]+1+j_hosu][k_hosu].dist=daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*j_hosu)][q_z[head]][k_hosu];
								daiRow[q_x[head]][q_y[head]+1+j_hosu][k_hosu].dire=0;
								daiRow[q_x[head]][q_y[head]+1+j_hosu][k_hosu].st=3+(2*j_hosu)+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head];
								q_y[tail] = q_y[head]+1+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
							j_hosu++;
						}else{
							break;
						}
					}
				}
				if(((q_x[head]+1)<G_maxMAP)&&((G_Column[q_x[head]+1]&(1<<q_y[head]))!=(1<<q_y[head]))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if(daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=1;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
				if((G_Column[q_x[head]]&(1<<q_y[head]))!=(1<<q_y[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if(daiCol[q_x[head]][q_y[head]][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiCol[q_x[head]][q_y[head]][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiCol[q_x[head]][q_y[head]][k_hosu].dire=7;
							daiCol[q_x[head]][q_y[head]][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
			}else if(daiRow[q_x[head]][q_y[head]][q_z[head]].dire==1){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if(((q_x[head]+1+i_hosu)<G_maxMAP)
							&&((q_y[head]+j_hosu)<G_maxMAP)
							&&(G_Column[q_x[head]+1+i_hosu]&(1<<(q_y[head]+j_hosu)))!=(1<<(q_y[head]+j_hosu))){
							if(daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].dire=1;
								daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+1+i_hosu;
								q_y[tail] = q_y[head]+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
						if(((q_x[head]+i_hosu)<G_maxMAP)
							&&((q_y[head]+1+j_hosu)<G_maxMAP)
							&&(G_Row[q_y[head]+1+j_hosu]&(1<<(q_x[head]+i_hosu)))!=(1<<(q_x[head]+i_hosu))){
							if(daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].dire=1;
								daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+i_hosu;
								q_y[tail] = q_y[head]+1+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						j_hosu++;
					}
				}
				if(((q_y[head]+1)<G_maxMAP)&&((G_Row[q_y[head]+1]&(1<<q_x[head]))!=(1<<q_x[head]))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st%200)-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st%200)-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
				if((G_Column[q_x[head]]&(1<<q_y[head]))!=(1<<q_y[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]][q_y[head]][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st%200)-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiCol[q_x[head]][q_y[head]][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st)%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[(daiRow[q_x[head]][q_y[head]][q_z[head]].st%200)-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiCol[q_x[head]][q_y[head]][k_hosu].dire=7;
							daiCol[q_x[head]][q_y[head]][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]][q_y[head]][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiCol[q_x[head]][q_y[head]][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiCol[q_x[head]][q_y[head]][k_hosu].dire=7;
							daiCol[q_x[head]][q_y[head]][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
			}else if(daiRow[q_x[head]][q_y[head]][q_z[head]].dire==7){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if((0<=(q_x[head]-i_hosu))
							&&((q_y[head]+j_hosu)<G_maxMAP)
							&&(G_Column[q_x[head]-i_hosu]&(1<<(q_y[head]+j_hosu)))!=(1<<(q_y[head]+j_hosu))){
							if(daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].dire=7;
								daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-i_hosu;
								q_y[tail] = q_y[head]+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						if((0<=q_x[head]-1-i_hosu)
							&&((q_y[head]+1+j_hosu)<G_maxMAP)
							&&(G_Row[q_y[head]+1+j_hosu]&(1<<(q_x[head]-1-i_hosu)))!=(1<<(q_x[head]-1-i_hosu))){
							if(daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[2+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[2+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].dire=7;
								daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].st=2+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-1-i_hosu;
								q_y[tail] = q_y[head]+1+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
						j_hosu++;
					}
				}
				if(((q_y[head]+1)<G_maxMAP)&&((G_Row[q_y[head]+1]&(1<<q_x[head]))!=(1<<q_x[head]))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=0;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
				if(((q_x[head]+1)<G_maxMAP)
					&&(G_Column[q_x[head]+1]&(1<<q_y[head]))!=(1<<q_y[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=1;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=1;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
			}else if(daiRow[q_x[head]][q_y[head]][q_z[head]].dire==4){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					j_hosu=0;
					while(1){
						if((0<=(q_y[head]-1-j_hosu))&&((G_Row[q_y[head]-1-j_hosu]&(1<<q_x[head]))!=(1<<q_x[head]))){
							if(daiRow[q_x[head]][q_y[head]-1-j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*j_hosu)][q_z[head]][k_hosu])){
								daiRow[q_x[head]][q_y[head]-1-j_hosu][k_hosu].dist=daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*j_hosu)][q_z[head]][k_hosu];
								daiRow[q_x[head]][q_y[head]-1-j_hosu][k_hosu].dire=4;
								daiRow[q_x[head]][q_y[head]-1-j_hosu][k_hosu].st=3+(2*j_hosu)+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head];
								q_y[tail] = q_y[head]-1-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
							j_hosu++;
						}else{
							break;
						}
					}
					if(((q_x[head]+1)<G_maxMAP)&&((G_Column[q_x[head]+1]&(1<<(q_y[head]-1)))!=(1<<(q_y[head]-1)))){
						if(daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dire=3;
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
					if((G_Column[q_x[head]]&(1<<(q_y[head]-1)))!=(1<<(q_y[head]-1))){
						if(daiCol[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiCol[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiCol[q_x[head]][q_y[head]-1][k_hosu].dire=5;
							daiCol[q_x[head]][q_y[head]-1][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
			}else if(daiRow[q_x[head]][q_y[head]][q_z[head]].dire==3){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if(((q_x[head]+1+i_hosu)<G_maxMAP)
							&&(0<=(q_y[head]-1-j_hosu))
							&&(G_Column[q_x[head]+1+i_hosu]&(1<<(q_y[head]-1-j_hosu)))!=(1<<(q_y[head]-1-j_hosu))){
							if(daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].dire=3;
								daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+1+i_hosu;
								q_y[tail] = q_y[head]-1-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
						j_hosu++;
						if(((q_x[head]+i_hosu)<G_maxMAP)
							&&(0<=(q_y[head]-j_hosu))
							&&(G_Row[q_y[head]-j_hosu]&(1<<(q_x[head]+i_hosu)))!=(1<<(q_x[head]+i_hosu))){
							if(daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].dire=3;
								daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].st=i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+i_hosu;
								q_y[tail] = q_y[head]-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
					}
				}
				if((G_Row[q_y[head]-1]&(1<<q_x[head]))!=(1<<q_x[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
				if((G_Column[q_x[head]]&(1<<(q_y[head]-1)))!=(1<<(q_y[head]-1))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiCol[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiCol[q_x[head]][q_y[head]-1][k_hosu].dire=5;
							daiCol[q_x[head]][q_y[head]-1][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiCol[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiCol[q_x[head]][q_y[head]-1][k_hosu].dire=5;
							daiCol[q_x[head]][q_y[head]-1][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
			}else if(daiRow[q_x[head]][q_y[head]][q_z[head]].dire==5){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if((0<=(q_x[head]-i_hosu))
							&&(0<=(q_y[head]-1-j_hosu))
							&&(G_Column[q_x[head]-i_hosu]&(1<<(q_y[head]-1-j_hosu)))!=(1<<(q_y[head]-1-j_hosu))){
							if(daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].dire=5;
								daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-i_hosu;
								q_y[tail] = q_y[head]-1-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						j_hosu++;
						if((0<=(q_x[head]-1-i_hosu))
							&&(0<=(q_y[head]-j_hosu))
							&&(G_Row[q_y[head]-j_hosu]&(1<<(q_x[head]-1-i_hosu)))!=(1<<(q_x[head]-1-i_hosu))){
							if(daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].dire=5;
								daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-1-i_hosu;
								q_y[tail] = q_y[head]-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
					}
				}
				if((G_Row[q_y[head]-1]&(1<<q_x[head]))!=(1<<q_x[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiRow[q_x[head]][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]-1][k_hosu].dire=4;
							daiRow[q_x[head]][q_y[head]-1][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
				if(((q_x[head]+1)<G_maxMAP)&&(G_Column[q_x[head]+1]&(1<<(q_y[head]-1)))!=(1<<(q_y[head]-1))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiRow[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiRow[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiRow[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dire=3;
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiRow[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dist>(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dist=(daiRow[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiRow[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].dire=3;
							daiCol[q_x[head]+1][q_y[head]-1][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head]-1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
			}
		}else if((q_kabe[head]==1)&&(0<q_x[head])){
			if(daiCol[q_x[head]][q_y[head]][q_z[head]].dire==2){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					while(1){
						if(((q_x[head]+1+i_hosu)<G_maxMAP)&&((G_Column[q_x[head]+1+i_hosu]&(1<<q_y[head]))!=(1<<q_y[head]))){
							if(daiCol[q_x[head]+1+i_hosu][q_y[head]][k_hosu].dist>daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*i_hosu)][q_z[head]][k_hosu]){
								daiCol[q_x[head]+1+i_hosu][q_y[head]][k_hosu].dist=daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*i_hosu)][q_z[head]][k_hosu];
								daiCol[q_x[head]+1+i_hosu][q_y[head]][k_hosu].dire=2;
								daiCol[q_x[head]+1+i_hosu][q_y[head]][k_hosu].st=3+(2*i_hosu)+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+1+i_hosu;
								q_y[tail] = q_y[head];
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
							i_hosu++;
						}else{
							break;
						}
					}
					if(((q_y[head]+1)<G_maxMAP)&&((G_Row[q_y[head]+1]&(1<<q_x[head]))!=(1<<q_x[head]))){
						if(daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=1;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
					if((G_Row[q_y[head]]&(1<<q_x[head]))!=(1<<q_x[head])){
						if(daiRow[q_x[head]][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiRow[q_x[head]][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiRow[q_x[head]][q_y[head]][k_hosu].dire=3;
							daiRow[q_x[head]][q_y[head]][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
			}else if(daiCol[q_x[head]][q_y[head]][q_z[head]].dire==1){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if(((q_x[head]+i_hosu)<G_maxMAP)
							&&((q_y[head]+1+j_hosu)<G_maxMAP)
							&&(G_Row[q_y[head]+1+j_hosu]&(1<<(q_x[head]+i_hosu)))!=(1<<(q_x[head]+i_hosu))){
							if(daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].dire=1;
								daiRow[q_x[head]+i_hosu][q_y[head]+1+j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+i_hosu;
								q_y[tail] = q_y[head]+1+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						j_hosu++;
						if(((q_x[head]+1+i_hosu)<G_maxMAP)
							&&((q_y[head]+j_hosu)<G_maxMAP)
							&&(G_Column[q_x[head]+1+i_hosu]&(1<<(q_y[head]+j_hosu)))!=(1<<(q_y[head]+j_hosu))){
							if(daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].dire=1;
								daiCol[q_x[head]+1+i_hosu][q_y[head]+j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+1+i_hosu;
								q_y[tail] = q_y[head]+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
					}
				}
				if(((q_x[head]+1)<G_maxMAP)&&((G_Column[q_x[head]+1]&(1<<q_y[head]))!=(1<<q_y[head]))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st==-10)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>omomi_startTR90[k_hosu]+omomi_ST[1][k_hosu][k_hosu]){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=omomi_startTR90[k_hosu]+omomi_ST[1][k_hosu][k_hosu];
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
				if((G_Row[q_y[head]]&(1<<q_x[head]))!=(1<<q_x[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiRow[q_x[head]][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiRow[q_x[head]][q_y[head]][k_hosu].dire=3;
							daiRow[q_x[head]][q_y[head]][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiRow[q_x[head]][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiRow[q_x[head]][q_y[head]][k_hosu].dire=3;
							daiRow[q_x[head]][q_y[head]][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st==-10)
							&&daiRow[q_x[head]][q_y[head]][k_hosu].dist>omomi_startTR135[k_hosu]){

							daiRow[q_x[head]][q_y[head]][k_hosu].dist=omomi_startTR135[k_hosu];
							daiRow[q_x[head]][q_y[head]][k_hosu].dire=3;
							daiRow[q_x[head]][q_y[head]][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
			}else if(daiCol[q_x[head]][q_y[head]][q_z[head]].dire==3){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if(((q_x[head]+i_hosu)<G_maxMAP)
							&&(0<=(q_y[head]-j_hosu))
							&&(G_Row[q_y[head]-j_hosu]&(1<<(q_x[head]+i_hosu)))!=(1<<(q_x[head]+i_hosu))){
							if(daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].dire=3;
								daiRow[q_x[head]+i_hosu][q_y[head]-j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+i_hosu;
								q_y[tail] = q_y[head]-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						if(((q_x[head]+1+i_hosu)<G_maxMAP)
							&&(0<=(q_y[head]-1-j_hosu))
							&&(G_Column[q_x[head]+1+i_hosu]&(1<<(q_y[head]-1-j_hosu)))!=(1<<(q_y[head]-1-j_hosu))){
							if(daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[2+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[2+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].dire=3;
								daiCol[q_x[head]+1+i_hosu][q_y[head]-1-j_hosu][k_hosu].st=2+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]+1+i_hosu;
								q_y[tail] = q_y[head]-1-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
						j_hosu++;
					}
				}
				if(((q_x[head]+1)<G_maxMAP)&&((G_Column[q_x[head]+1]&(1<<q_y[head]))!=(1<<q_y[head]))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_y[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_y[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_y[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiCol[q_x[head]+1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]+1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]+1][q_y[head]][k_hosu].dire=2;
							daiCol[q_x[head]+1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]+1;
							q_y[tail] = q_y[head];
							q_y[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
				if(((q_y[head]+1)<G_maxMAP)
					&&(G_Row[q_y[head]+1]&(1<<q_x[head]))!=(1<<q_x[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=1;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]][q_y[head]+1][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[q_z[head]][k_hosu]+omomi_TR135in[k_hosu][k_hosu])){

							daiRow[q_x[head]][q_y[head]+1][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[q_z[head]][k_hosu]+omomi_TR135in[k_hosu][k_hosu]);
							daiRow[q_x[head]][q_y[head]+1][k_hosu].dire=1;
							daiRow[q_x[head]][q_y[head]+1][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head];
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
			}else if(daiCol[q_x[head]][q_y[head]][q_z[head]].dire==6){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					while(1){
						if((0<=(q_x[head]-1-i_hosu))&&((G_Column[q_x[head]-1-i_hosu]&(1<<q_y[head]))!=(1<<q_y[head]))){
							if(daiCol[q_x[head]-1-i_hosu][q_y[head]][k_hosu].dist>daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*i_hosu)][q_z[head]][k_hosu]){
								daiCol[q_x[head]-1-i_hosu][q_y[head]][k_hosu].dist=daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_ST[3+(2*i_hosu)][q_z[head]][k_hosu];
								daiCol[q_x[head]-1-i_hosu][q_y[head]][k_hosu].dire=6;
								daiCol[q_x[head]-1-i_hosu][q_y[head]][k_hosu].st=3+(2*i_hosu)+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-1-i_hosu;
								q_y[tail] = q_y[head];
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
							i_hosu++;
						}else{
							break;
						}
					}
					if(((q_y[head]+1)<G_maxMAP)
						&&((G_Row[q_y[head]+1]&(1<<(q_x[head]-1)))!=(1<<(q_x[head]-1)))){
						if(daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dire=7;
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
					if((G_Row[q_y[head]]&(1<<(q_x[head]-1)))!=(1<<(q_x[head]-1))){
						if(daiRow[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu])){
							daiRow[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_ST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45in[q_z[head]][k_hosu]);
							daiRow[q_x[head]-1][q_y[head]][k_hosu].dire=5;
							daiRow[q_x[head]-1][q_y[head]][k_hosu].st=-2-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
			}else if(daiCol[q_x[head]][q_y[head]][q_z[head]].dire==7){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if((0<=(q_x[head]-1-i_hosu))
							&&((q_y[head]+1+j_hosu)<G_maxMAP)
							&&(G_Row[q_y[head]+1+j_hosu]&(1<<(q_x[head]-1-i_hosu)))!=(1<<(q_x[head]-1-i_hosu))){
							if(daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].dire=7;
								daiRow[q_x[head]-1-i_hosu][q_y[head]+1+j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-1-i_hosu;
								q_y[tail] = q_y[head]+1+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
						j_hosu++;
						if((0<=(q_x[head]-i_hosu))
							&&((q_y[head]+j_hosu)<G_maxMAP)
							&&(G_Column[q_x[head]-i_hosu]&(1<<(q_y[head]+j_hosu)))!=(1<<(q_y[head]+j_hosu))){
							if(daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].dire=7;
								daiCol[q_x[head]-i_hosu][q_y[head]+j_hosu][k_hosu].st=i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-i_hosu;
								q_y[tail] = q_y[head]+j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
					}
				}
				if((G_Column[q_x[head]-1]&(1<<q_y[head]))!=(1<<q_y[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
				if((G_Row[q_y[head]]&(1<<(q_x[head]-1)))!=(1<<(q_x[head]-1))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiRow[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiRow[q_x[head]-1][q_y[head]][k_hosu].dire=5;
							daiRow[q_x[head]-1][q_y[head]][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiRow[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiRow[q_x[head]-1][q_y[head]][k_hosu].dire=5;
							daiRow[q_x[head]-1][q_y[head]][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
			}else if(daiCol[q_x[head]][q_y[head]][q_z[head]].dire==5){
				for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
					i_hosu=0;
					j_hosu=0;
					while(1){
						if((0<=(q_x[head]-1-i_hosu))
							&&(0<=(q_y[head]-j_hosu))
							&&(G_Row[q_y[head]-j_hosu]&(1<<(q_x[head]-1-i_hosu)))!=(1<<(q_x[head]-1-i_hosu))){
							if(daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[1+i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].dire=5;
								daiRow[q_x[head]-1-i_hosu][q_y[head]-j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-1-i_hosu;
								q_y[tail] = q_y[head]-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=0;
								tail++;
							}
						}else{
							break;
						}
						i_hosu++;
						if((0<=(q_x[head]-i_hosu))
							&&(0<=(q_y[head]-1-j_hosu))
							&&(G_Column[q_x[head]-i_hosu]&(1<<(q_y[head]-1-j_hosu)))!=(1<<(q_y[head]-1-j_hosu))){
							if(daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[i_hosu+j_hosu][q_z[head]][k_hosu])){
								daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist+omomi_naST[i_hosu+j_hosu][q_z[head]][k_hosu]);
								daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].dire=5;
								daiCol[q_x[head]-i_hosu][q_y[head]-1-j_hosu][k_hosu].st=1+i_hosu+j_hosu+200*(3*q_z[head]+k_hosu);
								q_x[tail] = q_x[head]-i_hosu;
								q_y[tail] = q_y[head]-1-j_hosu;
								q_z[tail] = k_hosu;
								q_kabe[tail]=1;
								tail++;
							}
						}else{
							break;
						}
						j_hosu++;
					}
				}
				if((G_Column[q_x[head]-1]&(1<<q_y[head]))!=(1<<q_y[head])){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TR45out[q_z[head]][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-3)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR180[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-4)
							&&daiCol[q_x[head]-1][q_y[head]][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu])){

							daiCol[q_x[head]-1][q_y[head]][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TRv90[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135out[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]+omomi_ST[1][k_hosu][k_hosu]);
							daiCol[q_x[head]-1][q_y[head]][k_hosu].dire=6;
							daiCol[q_x[head]-1][q_y[head]][k_hosu].st=1+200*4*k_hosu;
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head];
							q_z[tail] = k_hosu;
							q_kabe[tail]=1;
							tail++;
						}
					}
				}
				if(((q_y[head]+1)<G_maxMAP)
					&&(G_Row[q_y[head]+1]&(1<<(q_x[head]-1)))!=(1<<(q_x[head]-1))){
					for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
						if((daiCol[q_x[head]][q_y[head]][q_z[head]].st>0)
							&&daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu])){

							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-(omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]]-omomi_naST[daiCol[q_x[head]][q_y[head]][q_z[head]].st%200-1][(daiCol[q_x[head]][q_y[head]][q_z[head]].st)/600][q_z[head]])+omomi_TRv90[q_z[head]][k_hosu]);
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dire=7;
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].st=-4-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}else if((daiCol[q_x[head]][q_y[head]][q_z[head]].st%10==-2)
							&&daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dist>(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu])){

							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dist=(daiCol[q_x[head]][q_y[head]][q_z[head]].dist-omomi_TR45in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][q_z[head]]+omomi_TR135in[(-daiCol[q_x[head]][q_y[head]][q_z[head]].st)/30][k_hosu]);
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].dire=7;
							daiRow[q_x[head]-1][q_y[head]+1][k_hosu].st=-3-10*(3*q_z[head]+k_hosu);
							q_x[tail] = q_x[head]-1;
							q_y[tail] = q_y[head]+1;
							q_z[tail] = k_hosu;
							q_kabe[tail]=0;
							tail++;
						}
					}
				}
			}
		}
		if(tail==head){
//			for(j_hosu=16;j_hosu>=0;j_hosu--){
//				printf("+");
//				for(i_hosu=0;i_hosu<=15;i_hosu++){
//					if((G_Row[j_hosu]&(1<<i_hosu))==(1<<i_hosu)){
//						printf("----+");
//					}else{
//						printf("%4d+",((int)((daiRow[i_hosu][j_hosu].dist)*(0.01))));
//					}
//				}
//				printf("\r\n");
//				for(i_hosu=0;i_hosu<=15;i_hosu++){
//					if(j_hosu!=0){
//						if((G_Column[i_hosu]&(1<<(j_hosu-1)))==(1<<(j_hosu-1))){
//							printf("|    ");
//						}else{
//							printf("%-5d",((int)((daiCol[i_hosu][j_hosu-1].dist)*(0.01))));
//						}
//					}
//				}
//				printf("|");
//				printf("\r\n");
//			}
//			printf("%d  %d",G_direction,G_direction%360);
//			printf("\r\n");
//			printf("%d  %d %d",G_goal_x,G_goal_y,G_goal_s);
//			printf("\r\n");
//			printf("Col %d",daiCol[1][1].dist/100 );
//			printf("\r\n");
			break;
		}
		head++;
		if(tail>G_maxMAP*G_maxMAP){
			for(i_hosu=0;i_hosu<(tail-head);i_hosu++){
				q_x[i_hosu]=q_x[i_hosu+head];
				q_y[i_hosu]=q_y[i_hosu+head];
				q_z[i_hosu]=q_z[i_hosu+head];
				q_kabe[i_hosu]=q_kabe[i_hosu+head];
			}
			printf("head=%d, tail=%d\n\r",head, tail);	//デバック
			tail=tail-head;
			head=0;
		}
	}
}

void set_hosu_dai(){//:初期化，初め〇
	pl_hosu_set();
	set_goal_daiku();
	while(1){
		//右上をゴールとした場合ゴールをlookpoitionとして終る。
		if(x_daiku==0&&y_daiku==0){
			break;
		}
		if(kabe_daiku==0){
			if(daiRow[x_daiku][y_daiku][z_daiku].st>0){
				j_hosu=daiRow[x_daiku][y_daiku][z_daiku].st%200;
				if(daiRow[x_daiku][y_daiku][z_daiku].dire==4){
					z_daiku=daiRow[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<(j_hosu/2)+1;i_hosu++){
						G_dist[x_daiku][y_daiku+1]=G_dist[x_daiku][y_daiku]+1;
						y_daiku++;
					}
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==0){
					z_daiku=daiRow[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<(j_hosu/2)+1;i_hosu++){
						G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
						y_daiku--;
					}
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==5){
					z_daiku=daiRow[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==0){
							G_dist[x_daiku+1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
							x_daiku++;
						}else{
							G_dist[x_daiku][y_daiku+1]=G_dist[x_daiku][y_daiku]+1;
							y_daiku++;
						}
					}
					kabe_daiku=1-(i_hosu%2);
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==3){
					z_daiku=daiRow[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==1){
							G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
							G_dist[x_daiku-1][y_daiku+1]=G_dist[x_daiku][y_daiku]+2;
							x_daiku--;
							y_daiku++;
						}
					}
					kabe_daiku=1-(i_hosu%2);
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==7){
					z_daiku=daiRow[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==0){
							G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
							G_dist[x_daiku+1][y_daiku-1]=G_dist[x_daiku][y_daiku]+2;
							x_daiku++;
							y_daiku--;
						}
					}
					kabe_daiku=1-(i_hosu%2);
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==1){
					z_daiku=daiRow[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==0){
							G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
							y_daiku--;
						}else{
							G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
							x_daiku--;
						}
					}
					kabe_daiku=1-(i_hosu%2);
				}
			}else{
				if(daiRow[x_daiku][y_daiku][z_daiku].dire==5){
					z_daiku=-(daiRow[x_daiku][y_daiku][z_daiku].st/30);
					G_dist[x_daiku+1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
					x_daiku++;
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==7){
					z_daiku=-(daiRow[x_daiku][y_daiku][z_daiku].st/30);
					G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
					G_dist[x_daiku+1][y_daiku-1]=G_dist[x_daiku][y_daiku]+2;
					x_daiku++;
					y_daiku--;
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==1){
					z_daiku=-(daiRow[x_daiku][y_daiku][z_daiku].st/30);
					G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
					y_daiku--;
				}else if(daiRow[x_daiku][y_daiku][z_daiku].dire==3){
					z_daiku=-(daiRow[x_daiku][y_daiku][z_daiku].st/30);
				}
				kabe_daiku=1;
			}
		}else if(kabe_daiku==1){
			if(daiCol[x_daiku][y_daiku][z_daiku].st>0){
				j_hosu=daiCol[x_daiku][y_daiku][z_daiku].st%200;
				if(daiCol[x_daiku][y_daiku][z_daiku].dire==6){
					z_daiku=daiCol[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<(j_hosu/2)+1;i_hosu++){
						G_dist[x_daiku+1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
						x_daiku++;
					}
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==2){
					z_daiku=daiCol[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<(j_hosu/2)+1;i_hosu++){
						G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
						x_daiku--;
					}
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==5){
					z_daiku=daiCol[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==0){
							G_dist[x_daiku][y_daiku+1]=G_dist[x_daiku][y_daiku]+1;
							y_daiku++;
						}else{
							G_dist[x_daiku+1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
							x_daiku++;
						}
					}
					kabe_daiku=(i_hosu%2);
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==3){
					z_daiku=daiCol[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==0){
							G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
							G_dist[x_daiku-1][y_daiku+1]=G_dist[x_daiku][y_daiku]+2;
							x_daiku--;
							y_daiku++;
						}
					}
					kabe_daiku=(i_hosu%2);
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==7){
					z_daiku=daiCol[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==1){
							G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
							G_dist[x_daiku+1][y_daiku-1]=G_dist[x_daiku][y_daiku]+2;
							x_daiku++;
							y_daiku--;
						}
					}
					kabe_daiku=(i_hosu%2);
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==1){
					z_daiku=daiCol[x_daiku][y_daiku][z_daiku].st/600;
					for(i_hosu=0;i_hosu<j_hosu;i_hosu++){
						if(i_hosu%2==0){
							G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
							x_daiku--;
						}else{
							G_dist[x_daiku][y_daiku-1]=G_dist[x_daiku][y_daiku]+1;
							y_daiku--;
						}
					}
					kabe_daiku=(i_hosu%2);
				}
			}else{
				if(daiCol[x_daiku][y_daiku][z_daiku].dire==5){
					z_daiku=-(daiCol[x_daiku][y_daiku][z_daiku].st/30);
					G_dist[x_daiku][y_daiku+1]=G_dist[x_daiku][y_daiku]+1;
					y_daiku++;
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==3){
					z_daiku=-(daiCol[x_daiku][y_daiku][z_daiku].st/30);
					G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
					G_dist[x_daiku-1][y_daiku+1]=G_dist[x_daiku][y_daiku]+2;
					x_daiku--;
					y_daiku++;
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==7){
					z_daiku=-(daiCol[x_daiku][y_daiku][z_daiku].st/30);
				}else if(daiCol[x_daiku][y_daiku][z_daiku].dire==1){
					z_daiku=-(daiCol[x_daiku][y_daiku][z_daiku].st/30);
					G_dist[x_daiku-1][y_daiku]=G_dist[x_daiku][y_daiku]+1;
					x_daiku--;
				}
				kabe_daiku=0;
			}
		}
	}
}

void set_goal_daiku(){
	int daigoal_flag;
	if(G_goal_s==0){
		x_goal_dai=G_goal_x;
		y_goal_dai=G_goal_y;
	}else if(G_goal_s==1){
		x_goal_dai=G_goal_x-1;
		y_goal_dai=G_goal_y;
	}else if(G_goal_s==2){
		x_goal_dai=G_goal_x;
		y_goal_dai=G_goal_y-1;
	}else if(G_goal_s==3){
		x_goal_dai=G_goal_x-1;
		y_goal_dai=G_goal_y-1;
	}

	distgoal_mini=99999;
	for(k_hosu=0;k_hosu<G_maxV;k_hosu++){
		if(daiRow[x_goal_dai][y_goal_dai][k_hosu].dire!=100){
			if(daiRow[x_goal_dai][y_goal_dai][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiRow[x_goal_dai][y_goal_dai][k_hosu].dist;
				x_daiku=x_goal_dai;
				y_daiku=y_goal_dai;
				z_daiku=k_hosu;
				kabe_daiku=0;
				daigoal_flag=0;
				G_goal_s=0;
			}
		}
		if(daiRow[x_goal_dai+1][y_goal_dai][k_hosu].dire!=100){
			if(daiRow[x_goal_dai+1][y_goal_dai][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiRow[x_goal_dai+1][y_goal_dai][k_hosu].dist;
				x_daiku=x_goal_dai+1;
				y_daiku=y_goal_dai;
				z_daiku=k_hosu;
				kabe_daiku=0;
				daigoal_flag=0;
				G_goal_s=1;
			}
		}
		if(daiRow[x_goal_dai][y_goal_dai+2][k_hosu].dire!=100){
			if(daiRow[x_goal_dai][y_goal_dai+2][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiRow[x_goal_dai][y_goal_dai+2][k_hosu].dist;
				x_daiku=x_goal_dai;
				y_daiku=y_goal_dai+1;
				z_daiku=k_hosu;
				kabe_daiku=0;
				daigoal_flag=1;
				G_goal_s=2;
			}
		}
		if(daiRow[x_goal_dai+1][y_goal_dai+2][k_hosu].dire!=100){
			if(daiRow[x_goal_dai+1][y_goal_dai+2][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiRow[x_goal_dai+1][y_goal_dai+2][k_hosu].dist;
				x_daiku=x_goal_dai+1;
				y_daiku=y_goal_dai+1;
				z_daiku=k_hosu;
				kabe_daiku=0;
				daigoal_flag=1;
				G_goal_s=3;
			}
		}
		if(daiCol[x_goal_dai][y_goal_dai][k_hosu].dire!=100){
			if(daiCol[x_goal_dai][y_goal_dai][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiCol[x_goal_dai][y_goal_dai][k_hosu].dist;
				x_daiku=x_goal_dai;
				y_daiku=y_goal_dai;
				z_daiku=k_hosu;
				kabe_daiku=1;
				daigoal_flag=0;
				G_goal_s=0;
			}
		}
		if(daiCol[x_goal_dai][y_goal_dai+1][k_hosu].dire!=100){
			if(daiCol[x_goal_dai][y_goal_dai+1][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiCol[x_goal_dai][y_goal_dai+1][k_hosu].dist;
				x_daiku=x_goal_dai;
				y_daiku=y_goal_dai+1;
				z_daiku=k_hosu;
				kabe_daiku=1;
				daigoal_flag=0;
				G_goal_s=2;
			}
		}
		if(daiCol[x_goal_dai+2][y_goal_dai][k_hosu].dire!=100){
			if(daiCol[x_goal_dai+2][y_goal_dai][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiCol[x_goal_dai+2][y_goal_dai][k_hosu].dist;
				x_daiku=x_goal_dai+1;
				y_daiku=y_goal_dai;
				z_daiku=k_hosu;
				kabe_daiku=1;
				daigoal_flag=2;
				G_goal_s=1;
			}
		}
		if(daiCol[x_goal_dai+2][y_goal_dai+1][k_hosu].dire!=100){
			if(daiCol[x_goal_dai+2][y_goal_dai+1][k_hosu].dist<distgoal_mini){
				distgoal_mini=daiCol[x_goal_dai+2][y_goal_dai+1][k_hosu].dist;
				x_daiku=x_goal_dai+1;
				y_daiku=y_goal_dai+1;
				z_daiku=k_hosu;
				kabe_daiku=1;
				daigoal_flag=2;
				G_goal_s=3;
			}
		}
	}
	G_dist[x_daiku][y_daiku]=0;
	if(daigoal_flag==1){
		G_dist[x_daiku][y_daiku+1]=1;
	}else if(daigoal_flag==2){
		G_dist[x_daiku+1][y_daiku]=1;
	}
	if(distgoal_mini==99999){
		printf("no_goal_wall\n\r");
	}else{
		G_goal_x=x_daiku;
		G_goal_y=y_daiku;
		G_goal_z=z_daiku;
	}
}

void check_daiku(){
	if(G_goalpass_s[G_goal_number]==1){
		if((G_look_position_y<G_maxMAP+1)
			&&((G_Row[G_look_position_y+1]&(1<<G_look_position_x))==(1<<G_look_position_x))){
			do_dai_flag=1;
		}
	}else if(G_goalpass_s[G_goal_number]==2){
		if((G_Row[G_look_position_y]&(1<<G_look_position_x))==(1<<G_look_position_x)){
			do_dai_flag=1;
		}
	}else if(G_goalpass_s[G_goal_number]==3){
		if((G_look_position_x<G_maxMAP+1)
			&&((G_Column[G_look_position_x+1]&(1<<G_look_position_y))==(1<<G_look_position_y))){
			do_dai_flag=1;
		}
	}else if(G_goalpass_s[G_goal_number]==4){
		if((G_Column[G_look_position_x]&(1<<G_look_position_y))==(1<<G_look_position_y)){
			do_dai_flag=1;
		}
	}
}
