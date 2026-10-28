/*
 * Move_sinsura.c
 *
 *  Created on: Oct 8, 2025
 *      Author: shino
 */
#include "Move_sinsura.h"
#include "gpio.h"
#include "stdio.h"
#include "math.h"

#include "motor_seigyo.h"
#include "PL_sensor.h"
#include "Wall_Control.h"

#include "G_hensu.h"

int mode_sura;
int do_sura_flag;

void do_sura_check(float dwVm, float dmae_sura, float dato_sura, int dmode_sura){
	if(dmode_sura==0){
		if((sura_tyousei_flag==0)
			&&(G_kabeR_flag==1||G_kabekireR_flag==1)
			&&(dwVm>=0.0)){
			if((G_subX_suraR>=(dmae_sura+tohasira))){
				do_sura_flag=1;
			}
		}else if((sura_tyousei_flag==0)
			&&(G_kabeL_flag==1||G_kabekireL_flag==1)
			&&(dwVm<0.0)){
			if((G_subX_suraL>=(dmae_sura+tohasira))){
				do_sura_flag=1;
			}
		}else if(G_kabeFL_flag==1&&G_kabeFR_flag==1){
			if(G_saitan_mode<=1){
				if((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F){
					do_sura_flag=1;
				}
			}else if(G_saitan_mode==2){
				if((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F2){
					do_sura_flag=1;
				}
			}else if(G_saitan_mode==3||G_saitan_mode==4){
				if((g_sensor[3][0]+g_sensor[0][0])>=SURAKABE_F3){
					do_sura_flag=1;
				}
			}
		}else{
			if(G_X_Motor>=dmae_sura){
				do_sura_flag=1;
			}
		}
	}
	if((dmode_sura>0)&&(dmode_sura<=4)){
		if((sura_tyousei_flag==0)&&(G_kabekireR_flag==1)&&(dwVm>=0.0)){
			if(G_subX_suraR>=dmae_sura){
				do_sura_flag=1;
			}
		}else if((sura_tyousei_flag==0)&&(G_kabekireL_flag==1)&&(dwVm<0.0)){
			if(G_subX_suraL>=dmae_sura){
				do_sura_flag=1;
			}
		}else{								//前距離が小さいと下は反応しない
			if(dmode_sura==1||dmode_sura==3){
				if(G_X_Motor>=dmae_sura+90.0-tohasira+10.0){
					do_sura_flag=1;
				}
			}else if(dmode_sura==2||dmode_sura==4){
				if(G_X_Motor>=dmae_sura-tohasira_na+10.0){
					do_sura_flag=1;
				}
			}
		}
	}
}

void sinsura(float wVm, float wAmax, float wXall, float mae_sura, float ato_sura, int mode_sura, int a_step){
	//mode_sura 0探索，1st_st, 2na_st, 3st_na, 4na_na
	float wv1_sura, wv2_sura;
	float acc_sura, acc_sura2, sub_acc, wxall_sura;
	int kk2, k2_sura, k2_sura2;
	float ca1,ca2,ca3;
	int a_step2;
	int n_all;
	float ratio_sura;
	acc_sura=wAmax/a_step;
	acc_sura2=acc_sura;
	a_step2=a_step;

	wv1_sura=acc_sura*0.5*0.001*(a_step-1)*(a_step);
	wv2_sura=(wVm-(wv1_sura*2));
	k2_sura=wv2_sura/(wAmax*0.001);
	k2_sura2=k2_sura;
	if((wv2_sura!=k2_sura*(wAmax*0.001))||(k2_sura<0)){
		a_step2=a_step+1;
		k2_sura2=k2_sura-1;
		if(k2_sura2<0){		//最大加速度に届かない場合
			k2_sura2=0;
		}
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

	a_step2=a_step;
	acc_sura2=acc_sura;
	wv1_sura=acc_sura*0.5*0.001*(a_step-1)*(a_step);
	wv2_sura=(wVm-(wv1_sura*2));
	k2_sura=wv2_sura/(wAmax*0.001);
	k2_sura2=k2_sura;
	if((wv2_sura!=k2_sura*(wAmax*0.001))||(k2_sura<0)){
		a_step2=a_step+1;
		k2_sura2=k2_sura-1;
		if(k2_sura2<0){		//最大加速度に届かない場合
			k2_sura2=0;
		}
		wv1_sura=wv1_sura+(wAmax*0.001);
		wv2_sura=wAmax*0.001*k2_sura2;
		acc_sura2=acc_sura*((wVm-wv2_sura)/(2*wv1_sura));
	}
	wxall_sura=wVm*(2*(a_step2-1)+k2_sura2)*0.001;
//	printf("hello %f\n\r",wxall_sura);

	kk2=((wXall-wxall_sura)/(wVm*0.001));
	if(kk2*(wVm*0.001)!=(wXall-wxall_sura)){
		kk2++;
	}
	if(kk2<0){		//分割加速の分だけでwXallを超える場合は等速区間なし
		kk2=0;
	}
	//it.cの加算で出る角度はwVm*総ステップ数*0.001になるので，速度と加速度全体に比率を掛けてぴったりwXallにする
	n_all=2*(a_step2-1)+k2_sura2+kk2;
	ratio_sura=wXall/(wVm*n_all*0.001);
	wVm=wVm*ratio_sura;
	wAmax=wAmax*ratio_sura;
	acc_sura2=acc_sura2*ratio_sura;
	sub_acc=wAmax-(a_step2-1)*acc_sura2;

	while(1){
//		do_sura_flag=1;
		do_sura_check(wVm, mae_sura, ato_sura, mode_sura);

		if(do_sura_flag==1){
			HAL_GPIO_WritePin(LED8_GPIO_Port, LED8_Pin, GPIO_PIN_SET);
			G_state=8;
			Change_PID();
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
					G_wVturn=0.0;
					G_wXturn=0.0;
					break;
				}
			}
			do_sura_flag=0;
			break;
		}
	}
	if(mode_sura<=2){
		G_state=1;
	}else{
		G_state=4;
	}
	Change_PID();
	G_X_Motor=0;
	sumEnc_L=0.0;
	sumEnc_R=0.0;
	reset_sensorbuff();
	sumGyro=0.0;
	while(1){
		if(G_X_Motor>=ato_sura){
			G_X_Motor=0.0;
			sumEnc_L=0.0;
			sumEnc_R=0.0;
			break;
		}
	}
}

//void sinsura(float wVm, float wAmax, float wXall, float mae_sura, float ato_sura, int mode_sura){
//	//mode_sura 0探索，1st_st, 2na_st, 3st_na, 4na_na
//	float wx_sura, t_sura;
//	wx_sura=(wVm*wVm*PI)/(4*wAmax);
//	t_sura=(PI*wVm)/(2*wAmax);
//
//	do_sura_flag=0;
//	if(wXall>wx_sura+wx_sura){
//		G_dtheta=PI/t_sura;
//	}else{
//		if(wVm>0.0){
//			wVm=sqrt((2*wAmax*wXall)*PI_);
//		}else{
//			wVm=-sqrt((2*wAmax*wXall)*PI_);
//		}
//		wx_sura=((wVm*wVm)*PI)/(4*wAmax);
//		t_sura=(PI*wVm)/(2*wAmax);
//		G_dtheta=PI/t_sura;
//	}
//
//
////	judge_kabe();
//	while(1){
//		do_sura_check(wVm, mae_sura, ato_sura, mode_sura);
//
//		if(do_sura_flag==1){
//			G_timer=0;
//			G_wXturn=0.0;
//			G_theta=0.0;
//			G_wAmax=wAmax;
//			sumGyro=0.0;
//
//			G_state=8;
//			Change_PID();
//			while(1){
//				if((((float)G_timer)/1000)>=(fabs(t_sura)-0.001)){
//					G_wAmax=0.0;
//					G_prewVturn = G_wVturn;
//					G_wVturn = wVm;
//					G_dtheta = 0.0;
//					G_theta = 0.0;
//					break;
//				}
//			}
//			while(1){
//				if((fabs(G_wXturn))>=fabs(wXall)-fabs(wx_sura)){
//					G_wAmax=wAmax;
//					G_dtheta=-(PI/t_sura);
//					break;
//				}
//			}
////			G_Pgain_Tur=G_Pgain_Tur*0.5;
////			G_Igain_Tur=G_Igain_Tur*0.5;
////			G_Dgain_Tur=G_Dgain_Tur*0.5;
//			G_timer=0;
//			while(1){
//				if((((float)G_timer)/1000)>=(fabs(t_sura)-0.001)){
//					G_wAmax=0.0;
//					G_prewVturn = G_wVturn;
//					G_wVturn=0.0;
//					G_wXturn=0.0;
//					G_dtheta=0.0;
//					G_theta=0.0;
//					sumGyro=0.0;
//					break;
//				}
//			}
//			do_sura_flag=0;
//			break;
//		}
//	}
//	if(mode_sura<=2){
//		G_state=1;
//	}else{
//		G_state=4;
//	}
//	Change_PID();
//	G_X_Motor=0;
//	sumEnc_L=0.0;
//	sumEnc_R=0.0;
//	reset_sensorbuff();
//	while(1){
//		if(G_X_Motor>=ato_sura){
//			G_X_Motor=0.0;
//			sumEnc_L=0.0;
//			sumEnc_R=0.0;
//			break;
//		}
//	}
//}

void sinsura2(float wVm, float wAmax, float wXall, float mae_sura, float ato_sura, int mode_sura, float Vstr, float Astr){
//	//mode_sura 0探索，1st_st, 2na_st, 3st_na, 4na_na
//	float wx_sura, t_sura;
//	wx_sura=(wVm*wVm*PI)/(4*wAmax);
//	t_sura=(PI*wVm)/(2*wAmax);
//
//	do_sura_flag=0;
//	if(wXall>wx_sura+wx_sura){
//		G_dtheta=PI/t_sura;
//	}else{
//		if(wVm>0.0){
//			wVm=sqrt((2*wAmax*wXall)*PI_);
//		}else{
//			wVm=-sqrt((2*wAmax*wXall)*PI_);
//		}
//		wx_sura=((wVm*wVm)*PI)/(4*wAmax);
//		t_sura=(PI*wVm)/(2*wAmax);
//		G_dtheta=PI/t_sura;
//	}
//
//	while(1){
//		do_sura_check(wVm, mae_sura, ato_sura, mode_sura);
//
//		if(do_sura_flag==1){
//			G_timer=0;
//			G_wXturn=0.0;
//			G_theta=0.0;
//			G_wAmax=wAmax;
//			sumGyro=0.0;
//
//			G_state=8;
//			Change_PID();
//			while(1){
//				if((((float)G_timer)/1000)>=(fabs(t_sura)-0.001)){
//					G_wAmax=0.0;
//					G_prewVturn = G_wVturn;
//					G_wVturn = wVm;
//					G_dtheta = 0.0;
//					G_theta = 0.0;
//					break;
//				}
//			}
//			while(1){
//				if((fabs(G_wXturn))>=fabs(wXall)-fabs(wx_sura)){
//					G_wAmax=wAmax;
//					G_dtheta=-(PI/t_sura);
//					break;
//				}
//			}
//			G_timer=0;
//			while(1){
//				if((((float)G_timer)/1000)>=(fabs(t_sura)-0.001)){
//					G_wAmax=0.0;
//					G_prewVturn = G_wVturn;
//					G_wVturn=0.0;
//					G_wXturn=0.0;
//					G_dtheta=0.0;
//					G_theta=0.0;
//					sumGyro=0.0;
//					break;
//				}
//			}
//			do_sura_flag=0;
//			break;
//		}
//	}
//	if(mode_sura<=2){
//		G_state=1;
//	}else{
//		G_state=4;
//	}
//	Change_PID();
//	G_X_Motor=0;
//	sumEnc_L=0.0;
//	sumEnc_R=0.0;
//	reset_sensorbuff();
//	if(Vstr>G_V_Motor){
//		kasoku(G_V_Motor, Vstr, Astr, ato_sura, 5, 5);
//	}else if(Vstr<G_V_Motor){
//		gensoku(G_V_Motor, Vstr, Astr, ato_sura, 5, 5);
//	}else if(Vstr==G_V_Motor){
//		renStr(ato_sura, 5, 5);
//	}
}

