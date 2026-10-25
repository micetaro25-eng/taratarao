/*
 * Wall_Control.h
 *
 *  Created on: Oct 28, 2025
 *      Author: shino
 */

#ifndef INC_WALL_CONTROL_H_
#define INC_WALL_CONTROL_H_

extern int G_kabeFL_flag;
extern int G_kabeFR_flag;
extern int G_kabeF_flag;
extern int G_kabeL_flag;
extern int G_kabeR_flag;
extern int G_kabekireL_flag;
extern int G_kabekireR_flag;

extern float G_subX_suraL;
extern float G_subX_suraR;

extern int G_WallControlStatus;
extern int G_WallState;

void judge_kabe();
void sensor_diff();

void kensa_wall_judge();
void PID_wall_judge();
void check_wall();

#endif /* INC_WALL_CONTROL_H_ */
