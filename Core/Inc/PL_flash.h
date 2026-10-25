/*
 * PL_flash.h
 *
 *  Created on: Dec 10, 2024
 *      Author: Ryu
 */

#ifndef INC_PL_FLASH_H_
#define INC_PL_FLASH_H_

#include "stm32f4xx_hal.h"
#include "Wall_record.h"
#include "G_hensu.h"

#define FLASH_SECTOR_SENSOR FLASH_SECTOR_6
#define FLASH_SECTOR_KABE FLASH_SECTOR_7

// 壁データ構造体（最大32）
typedef struct{
    int Row[G_maxMAP];
    int Column[G_maxMAP];
    int Wall_recorder_flag[G_maxMAP];
//	int Maze_M_Row[MAZESIZE];//3最短の横壁
//	int Maze_M_Column[MAZESIZE];//4最短の縦壁
//	uint64_t Maze_KnowMap[MAZE_SIZE];//5既知の区画かどうか(0:未知、1:既知)
} FLASH_WALL_DATA;

// センサデータ構造体（最大100）
typedef struct{
	int center_L, center_R, center_FL, center_FR, center_naFL, center_naFR;
	int threshold_L, threshold_R, threshold_FL, threshold_FR, threshold_naL, threshold_naR, threshold_naFL, threshold_naFR;
//	int threshold_dif_L, threshold_dif_R, threshold_dif_naL, threshold_dif_naR;
	int surakabe_F, surakabe_F2;
} FLASH_SENSOR_DATA;

// アドレス配置
#define START_ADR_SENSOR    0x08040000
#define START_ADR_WALL      0x08060000

// 関数プロトタイプ
void EraseSector(uint32_t sector);

void SaveFlashWall(FLASH_WALL_DATA* data);
void LoadFlashWall(FLASH_WALL_DATA* ret);

void SaveFlashSensor(FLASH_SENSOR_DATA* data);
void LoadFlashSensor(FLASH_SENSOR_DATA* ret);

void Save_wall();
void Load_wall();

void Save_sensor();
void Load_sensor();

void ResetFlashWall();

#endif /* INC_PL_FLASH_H_ */

