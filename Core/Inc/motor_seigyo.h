/*
 * motor_seigyo.h
 *
 *  Created on: Sep 20, 2025
 *      Author: shino
 */

#ifndef INC_MOTOR_SEIGYO_H_
#define INC_MOTOR_SEIGYO_H_

extern float G_V_maxst;
extern float G_V_maxtur1;
extern float G_V_maxtur2;
extern float G_V_comtur;
extern float G_A_maxst;
extern float G_A_comst;
extern float G_duty_L;
extern float G_duty_R;


void setVmax_Vcom(float,float,float,float,float,float);
void kasoku(float, float, float, float, int, int);
void gensoku(float, float, float, float, int, int);
void renStr(float, int, int);
void sakaku_daikei(float, float, float, float, float, int, int);
void sakaku_kaiten(float, float, float, int);
void gensoku_stop(float);
void Fix_X();

void set_CENTOR_sensor();
void calWallControl(int);
void judge_PID();

void Duty_Dire();
void DC_Motor_move();
void Change_PID();

#endif /* INC_MOTOR_SEIGYO_H_ */
