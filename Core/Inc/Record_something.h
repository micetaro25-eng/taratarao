/*
 * Record_something.h
 *
 *  Created on: Oct 15, 2025
 *      Author: shino
 */

#ifndef INC_RECORD_SOMETHING_H_
#define INC_RECORD_SOMETHING_H_

extern int sensor_log_flag;

void sensor_log();
void search_sen(int);
void reset_sen_log();
void display_sensor_log();

void log_Enc_Gyro();
void display_En_Gy();


#endif /* INC_RECORD_SOMETHING_H_ */
