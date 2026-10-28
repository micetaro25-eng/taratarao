/*
 * G_hensu.c
 *
 *  Created on: Sep 20, 2025
 *      Author: shino
 */

#include <stdint.h>
#include "G_hensu.h"

float hazimari=130.0;

//flag
int test_san_flag;
int motor_stop_flag;
int G_start_flag=0;
int G_doPID_flag;
int G_stopPID_flag;
int G_kabeate_flag;
int G_recostop_flag;
int do_dai_flag;
int G_kyuin_flag;
int sura_tyousei_flag;

int tohasi_log_flag;
int GV_log_flag;
int wV_log_flag;
int	Enc_log_flag;
int Gyr_log_flag;
int Gy_En_log_flag;
int G_logminSEN_L;
int G_logminSEN_R;
int G_logminSEN_FL;
int G_logminSEN_FR;
int G_logmaxSEN_FL;
int G_logmaxSEN_FR;
int G_logmaxdiff_L1;
int G_logmaxdiff_L2;
int G_logmaxdiff_R1;
int G_logmaxdiff_R2;
int	G_timer_log;

int Test_flag;

//変数
int G_state;		//0止まる, 1直進, 2回転, 3スラローム, 4斜め直進
int G_mode=0;
int G_LED_mode;		//0irumi(test用にしたい), 1,モード切替 2,壁 3,バッテリー
int G_saitan_mode=0;

//int G_inttimer=0;
int G_timer=0;
float G_theta=0.0;
float G_dtheta=0.0;
float G_Amax=0.0;
float G_A_Motor=0.0;
float G_a_Motor=0.0;		//分割加速用 1msごとの加速度の増分
float G_Abun_Motor=0.0;		//分割加速用 加速度
float G_preV_Motor=0.0;
float G_V_Motor=0.0;
float G_X_Motor=0.0;
float G_wAmax=0.0;
float G_waturn=0.0;
float G_wAturn=0.0;
float G_prewVturn=0.0;
float G_wVturn=0.0;
float G_wXturn=0.0;
float G_fix_X=0.0;
float G_PID_walltur=0.0;
float G_PID_wallstr=0.0;

float G_P_wall=0.0;
float G_Pgain_Str=0.0;		//から回しで0.0002,ついてると0.03
float G_Igain_Str=0.0;		//Pの100分の1くらい
float G_Dgain_Str=0.0;		//Pの10分の1くらい
float G_Pgain_Tur=0.0;
float G_Igain_Tur=0.0;
float G_Dgain_Tur=0.0;
float G_vFgain_Str=0.0;
float G_vFgain_Tur=0.0;
float G_aFgain_Str=0.0;
float G_aFgain_Tur=0.0;

//float g_Pe_KbSt=0.022;
//float g_Ie_KbSt=0.00065;
//float g_De_KbSt=0.002;
//float g_Pj_KbSt=0.015;
//float g_Ij_KbSt=0.0001;
//float g_Dj_KbSt=0.003;
//float g_FV_KbSt=0.00085;
//float g_FA_KbSt=0.0001;

float g_Pe_KbSt=0.022;
float g_Ie_KbSt=0.00065;
float g_De_KbSt=0.002;
float g_Pj_KbSt=0.014;
float g_Ij_KbSt=0.00009;
float g_Dj_KbSt=0.003;
float g_FV_KbSt=0.00085;
float g_FA_KbSt=0.00011;

//センサー
float g_V_batt;

float Gyro[2];
float sumGyro;

float Enc_L[2];
float Enc_R[2];
float sumEnc_L;
float sumEnc_R;

float tohasi_log;


//定数，調整

//int surakabe_F=550;
int sekuPIDwall_F=1000;
//int CENTER_L=260;
//int CENTER_R=250;
//int THRESHOLD_F=250;
//int THRESHOLD_L=90;
//int THRESHOLD_R=110;

//部室
int SURAKABE_F=580;
int SURAKABE_F2=450;
int SURAKABE_F3=450;
int CENTER_L=170;
int CENTER_R=390;
int CENTER_FL=1080;
int CENTER_FR=830;
int CENTER_naFL=30;
int CENTER_naFR=30;
int THRESHOLD_L=69;
int THRESHOLD_R=120;
int THRESHOLD_FL=80;
int THRESHOLD_FR=60;
int THRESHOLD_naL=1500;
int THRESHOLD_naR=1500;
int THRESHOLD_naFL=200;
int THRESHOLD_naFR=200;
int THRESHOLD_DIF_L=20;
int THRESHOLD_DIF_R=30;
int THRESHOLD_DIF_naL=300;
int THRESHOLD_DIF_naR=300;
//
//int SURAKABE_F=500;
//int SURAKABE_F2=380;
//int CENTER_L=347;
//int CENTER_R=250;
//int CENTER_FL=1030;
//int CENTER_FR=850;
//int CENTER_naFL=30;
//int CENTER_naFR=30;
//int THRESHOLD_L=150;
//int THRESHOLD_R=150;
//int THRESHOLD_FL=120;
//int THRESHOLD_FR=120;
//int THRESHOLD_naL=500;
//int THRESHOLD_naR=500;
//int THRESHOLD_naFL=1000;
//int THRESHOLD_naFR=1000;
//int THRESHOLD_DIF_L=40;
//int THRESHOLD_DIF_R=50;
//int THRESHOLD_DIF_naL=1000;
//int THRESHOLD_DIF_naR=1000;
//float surakabe_F=300;
//float CENTER_L=270.0;
//float CENTER_R=290.0;
//float THRESHOLD_DIF_L=1000.0;
//float THRESHOLD_DIF_R=1000.0;
