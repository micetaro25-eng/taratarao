/*
 * Wall_record.h
 *
 *  Created on: Oct 7, 2025
 *      Author: shino
 */

#ifndef INC_WALL_RECORD_H_
#define INC_WALL_RECORD_H_

#include "G_hensu.h"

extern int G_finish_flag;
extern int G_STnaname_flag;
extern int G_wall_recoder_flag[G_maxMAP];

extern int G_look_position_x;
extern int G_look_position_y;
extern int G_direction;

extern int G_firstgoal_x;
extern int G_firstgoal_y;
extern int G_goal_x;			//set goal
extern int G_goal_y;
extern int G_goal_z;
extern int G_goal_s;
extern int G_goalpass_x[G_maxMAP*G_maxMAP];
extern int G_goalpass_y[G_maxMAP*G_maxMAP];
extern int G_goalpass_s[G_maxMAP*G_maxMAP];
extern int G_goal_number;

extern int G_Column[G_maxMAP];
extern int G_Row[G_maxMAP];
extern int G_Column_save[G_maxMAP];
extern int G_Row_save[G_maxMAP];
extern int G_dist[G_maxMAP][G_maxMAP];

void Set_Wall();
void Set_Wall2();
void Set_Wall3();

void Record_Wall();
void Load_Wall();
void Save_Wall();

void display_wallhosu();
void display_recowall();
void display_goalpass();

#endif /* INC_WALL_RECORD_H_ */
