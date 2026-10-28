/*
 * G_hensu.h
 *
 *  Created on: Sep 20, 2025
 *      Author: shino
 */

#ifndef INC_G_HENSU_H_
#define INC_G_HENSU_H_

#include "math.h"

#define G_maxMAP 16
#define G_maxV 2
#define G_kukaku 180
#define G_noPIDwall 10
#define G_tansakumae 20
extern float hazimari;
#define kaitenC 1.09
#define tohasira 62.5		//壁キレしてから柱までの距離　半区画90
#define tohasira_na 10.0		//	壁キレしてから中央ライン(タイヤ)までの距離　半区画127.2
#define PI M_PI
#define PI_ (1.0/M_PI)
//ゲイン調整
//#define G_subPe_Str 0.03
//#define G_subIe_Str 0.00065
//#define G_subDe_Str 0.0003
//#define G_subPj_Str 0.01
//#define G_subIj_Str 0.0001
extern float g_Pe_KbSt;
extern float g_Ie_KbSt;
extern float g_De_KbSt;
extern float g_Pj_KbSt;
extern float g_Ij_KbSt;
extern float g_Dj_KbSt;
extern float g_FV_KbSt;
extern float g_FA_KbSt;
extern float g_FwV_KbSt;
extern float g_FwA_KbSt;

#define g_Pe_noKbSt 0.022;
#define g_Ie_noKbSt 0.00065;
#define g_De_noKbSt 0.002;
#define g_Pj_noKbSt 0.018;
#define g_Ij_noKbSt 0.0002;
#define g_Dj_noKbSt 0.0018;
#define g_FV_noKbSt 0.00085;
#define g_FA_noKbSt 0.00011;

#define g_Pe_Tur 0.013
#define g_Ie_Tur 0.00007
#define g_De_Tur 0.000
#define g_Pj_Tur 0.01	//部室
#define g_Ij_Tur 0.0006
#define g_Dj_Tur 0.0022
#define g_FV_Tur 0.0
#define g_FA_Tur 0.0
#define g_FwV_Tur 0.0026
#define g_FwA_Tur 0.0001

#define g_Pe_Sur 0.018
#define g_Ie_Sur 0.00011
#define g_De_Sur 0.002
#define g_Pj_Sur 0.014
#define g_Ij_Sur 0.0001
#define g_Dj_Sur 0.0038
#define g_FV_Sur 0.001
#define g_FA_Sur 0.00008
#define g_FwV_Sur 0.002
#define g_FwA_Sur 0.00009

#define g_Pe_KKbSt 0.03
#define g_Ie_KKbSt 0.001
#define g_De_KKbSt 0.01
#define g_Pj_KKbSt 0.02
#define g_Ij_KKbSt 0.0003
#define g_Dj_KKbSt 0.0000
#define g_FV_KKbSt 0.001
#define g_FA_KKbSt 0.00019

#define g_Pe_KnoKbSt 0.03
#define g_Ie_KnoKbSt 0.001
#define g_De_KnoKbSt 0.01
#define g_Pj_KnoKbSt 0.006
#define g_Ij_KnoKbSt 0.005
#define g_Dj_KnoKbSt 0.0000
#define g_FV_KnoKbSt 0.001
#define g_FA_KnoKbSt 0.00019

#define g_Pe_KSur 0.02
#define g_Ie_KSur 0.0005
#define g_De_KSur 0.004
#define g_Pj_KSur 0.017
#define g_Ij_KSur 0.0015
#define g_Dj_KSur 0.017
#define g_FV_KSur 0.0014
#define g_FA_KSur 0.00018
#define g_FwV_KSur 0.003
#define g_FwA_KSur 0.0001

#define g_Pe_B 0.01
#define g_Pj_B 0.005
#define g_FV_B 0.0
#define g_FA_B 0.0

//#define g_P_wall 0.000007
#define g_P_wall 0.000008
//#define g_P_nawall 0.000007
#define g_P_nawall 0.00007
#define g_P_maewall 0.0006
#define g_P_Kwall 0.000007
#define g_P_Knawall 0.00003

//#define G_Maxrate_hensu_tP 1.0	//0.1で，差分が目標値の0.1倍まで許容
//#define G_Maxrate_hensu_tI 0.15

#define omomi11_90 333
#define omomi11_180 666
#define omomi11_45in 0.35
#define omomi11_135in 0.15
#define omomi11_45out 0.15
#define omomi11_135out 0.15
#define omomi11_v90 0.15
#define omomi12_90 333
#define omomi12_180 666
#define omomi12_45in 99999
#define omomi12_135in 99999
#define omomi12_45out 99999
#define omomi12_135out 99999
#define omomi12_v90 99999
#define omomi21_90 333
#define omomi21_180 666
#define omomi21_45in 99999
#define omomi21_135in 99999
#define omomi21_45out 99999
#define omomi21_135out 99999
#define omomi21_v90 99999
#define omomi22_90 333
#define omomi22_180 666
#define omomi22_45in 99999
#define omomi22_135in 99999
#define omomi22_45out 99999
#define omomi22_135out 99999
#define omomi22_v90 99999

extern int G_mode_flag;
extern int test_san_flag;
extern int motor_stop_flag;
extern int G_start_flag;
extern int G_doPID_flag;
extern int G_stopPID_flag;
extern int G_kabeate_flag;
extern int G_recostop_flag;
extern int do_dai_flag;
extern int G_kyuin_flag;
extern int sura_tyousei_flag;
extern int tohasi_log_flag;
extern int GV_log_flag;
extern int wV_log_flag;
extern int Enc_log_flag;
extern int Gyr_log_flag;
extern int Gy_En_log_flag;
extern int G_logminSEN_L;
extern int G_logminSEN_R;
extern int G_logminSEN_FL;
extern int G_logminSEN_FR;
extern int G_logmaxSEN_FL;
extern int G_logmaxSEN_FR;
extern int G_logmaxdiff_L1;
extern int G_logmaxdiff_L2;
extern int G_logmaxdiff_R1;
extern int G_logmaxdiff_R2;
extern int G_timer_log;

extern int Test_flag;

extern int G_state;
extern int G_mode;
extern int G_LED_mode;
extern int G_saitan_mode;
//extern int G_inttimer;
extern int G_timer;
extern float G_A_Motor;
extern float G_a_Motor;
extern float G_preV_Motor;
extern float G_V_Motor;
extern float G_X_Motor;
extern float G_waturn;
extern float G_wAturn;
extern float G_prewVturn;
extern float G_wVturn;
extern float G_wXturn;
extern float G_fix_X;
extern float G_PID_walltur;
extern float G_PID_wallstr;
extern float G_P_wall;
extern float G_Pgain_Str;		//から回しで0.0002,ついてると0.03
extern float G_Igain_Str;		//Pの100分の1くらい
extern float G_Dgain_Str;		//Pの10分の1くらい
extern float G_Pgain_Tur;
extern float G_Igain_Tur;
extern float G_Dgain_Tur;
extern float G_vFgain_Str;
extern float G_vFgain_Tur;
extern float G_aFgain_Str;
extern float G_aFgain_Tur;

extern float g_V_batt;
extern float Gyro[2];
extern float sumGyro;
extern float Enc_L[2];
extern float Enc_R[2];
extern float sumEnc_L;
extern float sumEnc_R;
extern float tohasi_log;

extern int SURAKABE_F;
extern int SURAKABE_F2;
extern int SURAKABE_F3;
extern int sekuPIDwall_F;
extern int CENTER_L;
extern int CENTER_R;
extern int CENTER_FL;
extern int CENTER_FR;
extern int CENTER_naFL;
extern int CENTER_naFR;
extern int THRESHOLD_L;
extern int THRESHOLD_R;
extern int THRESHOLD_FL;
extern int THRESHOLD_FR;
extern int THRESHOLD_naL;
extern int THRESHOLD_naR;
extern int THRESHOLD_naFL;
extern int THRESHOLD_naFR;
extern int THRESHOLD_DIF_L;
extern int THRESHOLD_DIF_R;
extern int THRESHOLD_DIF_naL;
extern int THRESHOLD_DIF_naR;

#endif /* INC_G_HENSU_H_ */
