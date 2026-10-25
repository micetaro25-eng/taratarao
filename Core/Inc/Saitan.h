/*
 * Saitan.h
 *
 *  Created on: Oct 20, 2025
 *      Author: shino
 */

#ifndef INC_SAITAN_H_
#define INC_SAITAN_H_

extern int check_sula_flag;

extern float G_T_sula;
extern float G_timing_sula;

void trim_map_saitan();
void make_pass_saitan();

void move_sinsaitan();
void move_sinsaitan2();
void passcopy();
void move_kitisaitan();
void move_kitisaitan2();

void pass_assyuku();
void pass_assyuku_na();
void sin_pass_assyuku_na();
void pass_assyuku_sai();

void display_pass();

#endif /* INC_SAITAN_H_ */
