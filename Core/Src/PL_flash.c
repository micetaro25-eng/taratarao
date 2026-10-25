/*
 * PL_flash.c
 *
 *  Created on: Dec 22, 2025
 *      Author: shino
 */

#include "PL_flash.h"

// 任意セクタ消去
void EraseSector(uint32_t sector)
{
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Sector       = sector;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    uint32_t error = 0;
    HAL_FLASHEx_Erase(&erase, &error);
}

// 壁データ保存
void SaveFlashWall(FLASH_WALL_DATA* data)
{
    HAL_FLASH_Unlock();

    EraseSector(FLASH_SECTOR_KABE);
    // 壁データはセクタ先頭に書く
    uint32_t adr = START_ADR_WALL;
    uint32_t size = sizeof(FLASH_WALL_DATA);
    uint32_t wordCount = (size + 3) / 4;

    uint32_t* ptr = (uint32_t*)data;
    for(uint32_t i = 0; i < wordCount; i++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adr, *ptr);
        adr += 4;
        ptr++;
    }

    HAL_FLASH_Lock();
}

// 壁データ読み出し
void LoadFlashWall(FLASH_WALL_DATA* ret)
{
    *ret = *((FLASH_WALL_DATA*)START_ADR_WALL);
}

// センサデータ保存
void SaveFlashSensor(FLASH_SENSOR_DATA* data)
{
    HAL_FLASH_Unlock();
    EraseSector(FLASH_SECTOR_SENSOR);
    uint32_t adr = START_ADR_SENSOR;
    uint32_t size = sizeof(FLASH_SENSOR_DATA);
    uint32_t wordCount = (size + 3) / 4;

    uint32_t* ptr = (uint32_t*)data;
    for(uint32_t i = 0; i < wordCount; i++){
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, adr, *ptr);
        adr += 4;
        ptr++;
    }

    HAL_FLASH_Lock();
}

void LoadFlashSensor(FLASH_SENSOR_DATA* ret)
{
    *ret = *((FLASH_SENSOR_DATA*)START_ADR_SENSOR);
}

void Load_wall()
{
	FLASH_WALL_DATA flash_data;
	LoadFlashWall(&flash_data);
	for(int i_step_flash=0;i_step_flash<G_maxMAP;i_step_flash++){
		G_Row[i_step_flash]=flash_data.Row[i_step_flash];
		G_Column[i_step_flash]=flash_data.Column[i_step_flash];
		G_wall_recoder_flag[i_step_flash]=flash_data.Wall_recorder_flag[i_step_flash];
	}
}

void Save_wall()
{
	int i_step_flash;
	FLASH_WALL_DATA flash_data;
	for(i_step_flash=0;i_step_flash<G_maxMAP;i_step_flash++){
		flash_data.Row[i_step_flash]=G_Row[i_step_flash];
		flash_data.Column[i_step_flash]=G_Column[i_step_flash];
		flash_data.Wall_recorder_flag[i_step_flash]=G_wall_recoder_flag[i_step_flash];
	}
	SaveFlashWall(&flash_data);
}

// センサ読み出し（アプリ側）
void Load_sensor()
{
	FLASH_SENSOR_DATA flash_data;
	LoadFlashSensor(&flash_data);
	CENTER_L=flash_data.center_L;
	CENTER_R=flash_data.center_R;
	CENTER_FL=flash_data.center_FL;
	CENTER_FR=flash_data.center_FR;
//	CENTER_naFL=flash_data.sensor.center_naFL;
//	CENTER_naFR=flash_data.sensor.center_naFR;
	THRESHOLD_L=flash_data.threshold_L;
	THRESHOLD_R=flash_data.threshold_R;
	THRESHOLD_FL=flash_data.threshold_FL;
	THRESHOLD_FR=flash_data.threshold_FR;
	THRESHOLD_naL=flash_data.threshold_naL;
	THRESHOLD_naR=flash_data.threshold_naR;
//	THRESHOLD_naFL=flash_data.sensor.threshold_naFL;
//	THRESHOLD_naFR=flash_data.sensor.threshold_naFR;
//	THRESHOLD_DIF_L=flash_data.sensor.threshold_dif_L;
//	THRESHOLD_DIF_R=flash_data.sensor.threshold_dif_R;
//	THRESHOLD_DIF_naL=flash_data.sensor.threshold_dif_naL;
//	THRESHOLD_DIF_naR=flash_data.sensor.threshold_dif_naR;
	SURAKABE_F=flash_data.surakabe_F;
//	SURAWALL_F2=flash_data.sensor.suraWALL_F2;
}

// センサ保存（アプリ側）
void Save_sensor()
{
	FLASH_SENSOR_DATA flash_data;
	flash_data.center_L=CENTER_L;
	flash_data.center_R=CENTER_R;
	flash_data.center_FL=CENTER_FL;
	flash_data.center_FR=CENTER_FR;
//	flash_data.sensor.CENTER_naFL=CENTER_naFL;
//	flash_data.sensor.CENTER_naFR=CENTER_naFR;
	flash_data.threshold_L=THRESHOLD_L;
	flash_data.threshold_R=THRESHOLD_R;
	flash_data.threshold_FL=THRESHOLD_FL;
	flash_data.threshold_FR=THRESHOLD_FR;
	flash_data.threshold_naL=THRESHOLD_naL;
	flash_data.threshold_naR=THRESHOLD_naR;
//	flash_data.sensor.threshold_naFL=THRESHOLD_naFL;
//	flash_data.sensor.threshold_naFR=THRESHOLD_naFR;
//	flash_data.sensor.threshold_dif_L=THRESHOLD_DIF_L;
//	flash_data.sensor.threshold_dif_R=THRESHOLD_DIF_R;
//	flash_data.sensor.threshold_dif_naL=THRESHOLD_DIF_naL;
//	flash_data.sensor.threshold_dif_naR=THRESHOLD_DIF_naR;
	flash_data.surakabe_F=SURAKABE_F;
//	flash_data.sensor.surakabe_F2=SURAKABE_F2;
	SaveFlashSensor(&flash_data);
}

void ResetFlashWall()
{
	for(int i_step_flash=0;i_step_flash<G_maxMAP;i_step_flash++){
		G_Row[i_step_flash]=0;
		G_Column[i_step_flash]=0;
		G_wall_recoder_flag[i_step_flash]=0;
	}
	Set_Wall();
	Save_wall();
}
