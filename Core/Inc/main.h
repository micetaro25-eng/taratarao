/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Sensor_4_Pin GPIO_PIN_0
#define Sensor_4_GPIO_Port GPIOC
#define Sensor_6_Pin GPIO_PIN_1
#define Sensor_6_GPIO_Port GPIOC
#define Sensor_3_Pin GPIO_PIN_2
#define Sensor_3_GPIO_Port GPIOC
#define WallLED_NR_Pin GPIO_PIN_3
#define WallLED_NR_GPIO_Port GPIOC
#define FunPWM_Pin GPIO_PIN_0
#define FunPWM_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_1
#define Buzzer_GPIO_Port GPIOA
#define ST_TX_Pin GPIO_PIN_2
#define ST_TX_GPIO_Port GPIOA
#define BATT_Pin GPIO_PIN_3
#define BATT_GPIO_Port GPIOA
#define MPU6500_CS_Pin GPIO_PIN_4
#define MPU6500_CS_GPIO_Port GPIOA
#define WallLED_NL_Pin GPIO_PIN_4
#define WallLED_NL_GPIO_Port GPIOC
#define Sensor_5_Pin GPIO_PIN_5
#define Sensor_5_GPIO_Port GPIOC
#define Sensor_1_Pin GPIO_PIN_0
#define Sensor_1_GPIO_Port GPIOB
#define Sensor_2_Pin GPIO_PIN_1
#define Sensor_2_GPIO_Port GPIOB
#define WallLED_FR_L_Pin GPIO_PIN_2
#define WallLED_FR_L_GPIO_Port GPIOB
#define WallLED_FL_R_Pin GPIO_PIN_10
#define WallLED_FL_R_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_13
#define STBY_GPIO_Port GPIOB
#define MotorIN1_R_Pin GPIO_PIN_14
#define MotorIN1_R_GPIO_Port GPIOB
#define MotorIN2_R_Pin GPIO_PIN_15
#define MotorIN2_R_GPIO_Port GPIOB
#define Enc1_R_Pin GPIO_PIN_6
#define Enc1_R_GPIO_Port GPIOC
#define Enc2_R_Pin GPIO_PIN_7
#define Enc2_R_GPIO_Port GPIOC
#define MotorPWM_R_Pin GPIO_PIN_8
#define MotorPWM_R_GPIO_Port GPIOC
#define MotorPWM_L_Pin GPIO_PIN_9
#define MotorPWM_L_GPIO_Port GPIOC
#define Enc2_L_Pin GPIO_PIN_8
#define Enc2_L_GPIO_Port GPIOA
#define Enc1_L_Pin GPIO_PIN_9
#define Enc1_L_GPIO_Port GPIOA
#define MotorIN1_L_Pin GPIO_PIN_10
#define MotorIN1_L_GPIO_Port GPIOA
#define MotorIN2_L_Pin GPIO_PIN_11
#define MotorIN2_L_GPIO_Port GPIOA
#define LED5_Pin GPIO_PIN_10
#define LED5_GPIO_Port GPIOC
#define LED6_Pin GPIO_PIN_11
#define LED6_GPIO_Port GPIOC
#define LED7_Pin GPIO_PIN_12
#define LED7_GPIO_Port GPIOC
#define LED8_Pin GPIO_PIN_2
#define LED8_GPIO_Port GPIOD
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_6
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
