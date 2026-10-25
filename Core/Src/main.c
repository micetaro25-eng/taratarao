/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "PL_timer.h"
#include "PL_sensor.h"
#include "PL_ENC.h"
#include "PL_gyro.h"

#include "Mode.h"
#include "motor_seigyo.h"
#include "Wall_Control.h"
#include "Move_sinsura.h"
#include "move_preset.h"
#include "Wall_record.h"
#include "Set_goal.h"
#include "Adatimethod.h"
#include "Hosu_make.h"
#include "Saitan.h"

#include "PL_flash.h"
#include "Record_something.h"
#include "test_something.h"
#include "G_hensu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	pl_timer_init();
	mpu6500_init();
	ENC_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	stop_kyuin();
// STBYを有効化
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		main_Mode();

		if(G_mode==1){
////test gpio
//			tes_gpio();
////test USART1
//			tes_usart1();
////test USART2
//			tes_usart2();
////test timer
//			irumi_LED();
////test jyro
//			check_jyro();
//			check_jyro2();
//			tes_jyro();
////test speaker
//		tes_speaker();
			start_kyuin(20.0);
			wait_ms(1000);
			start_kyuin(25.0);
			wait_ms(300);
			start_kyuin(30.0);
			wait_ms(300);
			start_kyuin(35.0);
			wait_ms(1000);
			kasoku(0.0, 1500.0, 25000, hazimari, 4, 0);
			gensoku_stop(35000);
			wait_ms(1000);
			DC_motor_stop();
			stop_kyuin();
		}
		if(G_mode==2){
////test Sensor
//			set_CENTOR_sensor();
//			sakaku_daikei(0.0, 1500.0, 0.0, 10000, 2700);
			chenk_BATT();
		}
		if(G_mode==3){
//			G_LED_mode=2;
//			G_state=1;
//			Save_wall();
//			Load_wall();
			check_sensor();
			CENTER_L=3;
			printf("CENTER_L%d\n\r",CENTER_L);
			Load_sensor();
			printf("CENTER_L%d\n\r",CENTER_L);
		}
		if(G_mode==4){
			tes_sensor();
//			cary_fkou();
		}
		if(G_mode==5){
//			set_sensor();
//			tes_sensor();
////test sankakuG_V
//			tes_sankaku();
//			tes_sankaku2();
////test motor&Enc
//			tes_motor_Enc();
////check Enc
//			check_Enc();
////test Enc Gyro sankaku mot
//			start_kyuin(20);
//			tes_tyokusin();
//			tes_kaiten();
//////センサー調整用，スラロームの前壁，壁キレ，中央にいる時のセンサ値測定
////			tes_judgewall();
////			mpu6500_do_off();
////			set_CENTOR_sensor();
////			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari+180.0);
////			DC_motor_stop();
//
//			G_subP_wall=0.0;
//			tes_sendiff();
			Load_sensor();
			cary_sen();
			Save_sensor();
//			irumi_LED();
//////センサーの壁キレより細かく
////			tes_sendiff2();
//			G_mode_flag=0;
		}
		if(G_mode==6){
			Load_sensor();
//			tes_tyokusin();
//			tes_kyuintyoku();
//			tes_kaiten();
////test Sura
//			tes_sura(9);
//			tes_sura(1);
//			tes_sura(2);
//			tes_sura(3);
//			tes_sura(4);
//			tes_sura(5);
//			tes_sura(6);
//			tes_sura(7);
//			tes_sura(8);
//			tes_sura(10);
			tes_sura(11);
		}
		if(G_mode==7){
//daiku
//			Set_Wall2();
//			setVmax_Vcom(2800.0, 1000.0, 1200.0, 500.0, 12500.0, 10000.0);
//			display_wallhosu();
//			daiku_hosumake(0);
//			set_hosu_dai();
//			set_goal_dai();
			Load_wall();
			display_wallhosu();
//			display_goalpass();
		}
		if(G_mode==8){
//adati
//			adati_test();
//			move_kabeawase();
//			while(1){
//				wait_ms(100);
//			}
			sakaku_daikei(0.0, 500.0, 0.0, 10000.0, hazimari+9, 5, 5);
			DC_motor_stop();
			int aaaa=g_sensor[0][0]+g_sensor[3][0];
			irumi_LED();
			while(1){
				if((g_sensor[0][0]+g_sensor[3][0]>SURAKABE_F)){
					irumi_LED();
					printf("%d\n\r",aaaa);
				}
			}
		}
		if(G_mode==9){
			Load_wall();
			Load_sensor();
			Save_Wall();
			setVmax_Vcom(4000.0, 1500.0, 1500.0, 1300.0, 26500.0, 15000.0);
			G_saitan_mode=3;

			G_look_position_y=0;
			G_look_position_x=0;
			G_direction=1080;
			trim_map_saitan();
			reset_goalpass();
			set_goalpass_07();
			pl_hosu_set();
			pl_hosu_make();

			make_pass_saitan();
			display_pass();
			pass_assyuku_na();
			display_pass();
			display_wallhosu();
		}
		if(G_mode==10){
			Set_Wall3();
			Save_wall();
		}
		if(G_mode==11){
			flashtest2();
		}
		if(G_mode==12){
			irumi_LED();
			G_goal_x=8;
			G_goal_y=7;
			Set_Wall2();
			Save_Wall();

			trim_map_saitan();
			pl_hosu_set();
			pl_hosu_make();
			make_pass_saitan();
			display_wallhosu();
			pass_assyuku_na();
			display_pass();

			Load_Wall();
		}
		if(G_mode==13){
			tes_sandai(-1500.0, -40000.0, -45.0, 10);
		}
		if(G_mode==16){
			ResetFlashWall();

//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					setVmax_Vcom(1500.0, 1000.0, 500.0, 15000.0, 10000.0);	//直進
//					G_saitan_mode=0;										//ターン
////					G_LED_mode=2;
//					reset_goalpass();
//					set_goalpass_07();
////					sura_tyousei_flag=1;
//					Adatimethod(1);
//					Save_Wall();
//					break;
//				}
//			}
		}

		if(G_mode==17){				//既知区間無し，両面探索
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					setVmax_Vcom(1800.0, 500.0, 500.0, 500.0, 8000.0, 8000.0);	//直進
					G_saitan_mode=0;										//ターン
//					G_LED_mode=2;
					reset_goalpass();
					set_goalpass_07();
					Adatimethod();
					break;
				}
			}
		}
		if(G_mode==18){		//多分既知区間加速
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					setVmax_Vcom(1800.0, 500.0, 500.0, 500.0, 8000.0, 8000.0);	//直進
					G_saitan_mode=0;										//ターン
//					G_LED_mode=2;
					set_CENTOR_sensor();
					reset_goalpass();
					set_goalpass_07();
					Adatimethod_kiti();
					break;
				}
			}
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					setVmax_Vcom(1500.0, 1000.0, 1200.0, 500.0, 15000.0, 10000.0);	//直進
//					G_saitan_mode=0;										//ターン
////					G_LED_mode=2;
//					reset_goalpass();
//					set_goalpass_07();
//					Adatimethod_kiti();
//					break;
//				}
//			}
		}
		if(G_mode==19){												//両面ダイクストラ_kiti
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					setVmax_Vcom(2500.0, 1000.0, 1200.0, 500.0, 12500.0, 10000.0);	//直進
					G_saitan_mode=0;										//ターン
//					G_LED_mode=2;
					reset_goalpass();
					set_goalpass_07();
					Adatimethod_daikiti();
					break;
				}
			}
		}
//		if(G_mode==20){						//小回りオンリー最短
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					reset_goalpass();
//					set_goalpass_07();
//					setVmax_Vcom(2800.0, 1000.0, 1200.0 ,500.0, 12500.0, 10000.0);
//
//					G_saitan_mode=0;
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					pl_hosu_set();
//					pl_hosu_make();
//					make_pass_saitan();
//					move_sinsaitan();
//
//					Load_Wall();
//					break;
//				}
//			}
//		}
//
//		if(G_mode==21){						//大廻90，180
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					setVmax_Vcom(2800.0, 1000.0, 500.0, 12500.0, 10000.0);
//					G_saitan_mode=0;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					daiku_hosumake(0);
//					set_hosu_dai();
//					reset_goalpass();
//					set_goalpass_07();
////					display_wallhosu();
//					make_pass_saitan();
//					pass_assyuku();
//					move_sinsaitan();
//
//					Load_Wall();
//					break;
//				}
//			}
//		}

		if(G_mode==20){					//吸引90，180だけ
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					Load_wall();
					Load_sensor();
					Save_Wall();
					setVmax_Vcom(4000.0, 1500.0, 1500.0, 1200.0, 26500.0, 15000.0);
					G_saitan_mode=3;

					G_look_position_y=0;
					G_look_position_x=0;
					G_direction=1080;
					trim_map_saitan();
					reset_goalpass();
					set_goalpass_07();
					pl_hosu_set();
					pl_hosu_make();

					make_pass_saitan();
					pass_assyuku();

					mpu6500_do_off();
					start_kyuin(20.0);
					wait_ms(700);
					start_kyuin(25.0);
					wait_ms(300);
					start_kyuin(30.0);
					wait_ms(300);
					start_kyuin(35.0);
					wait_ms(1300);
					move_sinsaitan();
					stop_kyuin();
					Load_Wall();
					break;
				}
			}
		}
		if(G_mode==21){
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					Load_wall();
					Load_sensor();
					Save_Wall();
					setVmax_Vcom(4000.0, 1700.0, 1700.0, 1700.0, 32500.0, 15000.0);
					G_saitan_mode=4;

					G_look_position_y=0;
					G_look_position_x=0;
					G_direction=1080;
					trim_map_saitan();
					reset_goalpass();
					set_goalpass_07();
					pl_hosu_set();
					pl_hosu_make();
					make_pass_saitan();
					pass_assyuku();

					mpu6500_do_off();
					start_kyuin(20.0);
					wait_ms(1500);
					start_kyuin(25.0);
					wait_ms(300);
					start_kyuin(30.0);
					wait_ms(300);
					start_kyuin(35.0);
					wait_ms(1000);
					move_sinsaitan();
					stop_kyuin();
					Load_Wall();
					break;
				}
			}
		}
		if(G_mode==22){						//最短吸引小回り
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					Load_wall();
					Load_sensor();
					Save_Wall();
					setVmax_Vcom(4000.0, 1500.0, 1500.0, 1500.0, 26500.0, 15000.0);
					G_saitan_mode=3;

					G_look_position_y=0;
					G_look_position_x=0;
					G_direction=1080;
					trim_map_saitan();
					reset_goalpass();
					set_goalpass_07();
					pl_hosu_set();
					pl_hosu_make();
					make_pass_saitan();
					pass_assyuku();
					mpu6500_do_off();
					start_kyuin(20.0);
					wait_ms(1500);
					start_kyuin(25.0);
					wait_ms(500);
					start_kyuin(30.0);
					wait_ms(500);
					start_kyuin(35.0);
					wait_ms(1000);
					move_sinsaitan();
					stop_kyuin();
					Load_Wall();
					break;
				}
			}
		}
		if(G_mode==23){						//最短吸引ななめ
			while(1){
				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
					irumi_LED();
					Load_wall();
					Load_sensor();
					Save_Wall();
					setVmax_Vcom(4000.0, 1500.0, 1500.0, 1300.0, 26500.0, 15000.0);
					G_saitan_mode=3;

					G_look_position_y=0;
					G_look_position_x=0;
					G_direction=1080;
					trim_map_saitan();
					reset_goalpass();
					set_goalpass_07();
					pl_hosu_set();
					pl_hosu_make();

					make_pass_saitan();
					pass_assyuku_na();

					mpu6500_do_off();
					start_kyuin(20.0);
					wait_ms(700);
					start_kyuin(25.0);
					wait_ms(300);
					start_kyuin(30.0);
					wait_ms(300);
					start_kyuin(35.0);
					wait_ms(1300);
					move_sinsaitan();
					stop_kyuin();
					Load_Wall();
					break;
				}
			}
		}
		if(G_mode==21){
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					Load_wallsen();
//					Save_Wall();
//					setVmax_Vcom(4000.0, 1700.0, 2000.0, 850.0, 30000.0, 35000.0);
//					G_saitan_mode=1;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					reset_goalpass();
//					set_goalpass_07();
//					pl_hosu_set();
//					pl_hosu_make();
//					make_pass_saitan();
//					pass_assyuku_na();
//					mpu6500_do_off();
//					start_kyuin(20.0);
//					wait_ms(1500);
//					start_kyuin(25.0);
//					wait_ms(500);
//					start_kyuin(30.0);
//					wait_ms(1000);
//					move_sinsaitan2();
//					stop_kyuin();
//					Load_Wall();
//					break;
//				}
//			}
		}

		if(G_mode==23){
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					setVmax_Vcom(3000.0, 1000.0, 850.0, 15000.0, 15000.0);
//					G_saitan_mode=1;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					daiku_hosumake(0);
//					set_hosu_dai();
//					reset_goalpass();
//					set_goalpass_07();
//					make_pass_saitan();
////					pass_assyuku_na();
//					pass_assyuku();
//					move_sinsaitan();
//
//					Load_Wall();
//					break;
//				}
//			}
		}
//		if(G_mode==24){						//非吸引斜め
//			irumi_LED();
//			setVmax_Vcom(1500.0, 1000.0, 500.0, 15000.0, 8000.0);	//直進
//			G_saitan_mode=0;										//ターン
////			G_LED_mode=2;
//			reset_goalpass();
//			set_goalpass_07();
//			Adatimethod_daikiti();
//			Save_Wall();
//
//			mpu6500_do_off();
//			reset_goalpass();
//			set_goalpass_07();
//			G_goal_x=G_goalpass_x[0];
//			G_goal_y=G_goalpass_y[0];
//			setVmax_Vcom(2800.0, 1000.0, 500.0, 15000.0, 8000.0);
//
//			G_look_position_y=0;
//			G_look_position_x=0;
//			G_direction=1080;
//			trim_map_saitan();
//			daiku_hosumake();
//			set_hosu_dai();
//			make_pass_saitan();
//			pass_assyuku();
//			move_sinsaitan();
//			Load_Wall();
//			G_mode_flag=0;
//		}
		if(G_mode==24){
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					setVmax_Vcom(2800.0, 1000.0, 500.0, 15000.0, 10000.0);
//					G_saitan_mode=0;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					reset_goalpass();
//					set_goalpass_07();
//					pl_hosu_set();
//					pl_hosu_make();
//					make_pass_saitan();
//					pass_assyuku();
//					move_sinsaitan();
//
//					Load_Wall();
//					break;
//				}
//			}
		}
		if(G_mode==25){
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					setVmax_Vcom(2800.0, 1000.0, 500.0, 15000.0, 10000.0);
//					G_saitan_mode=0;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					reset_goalpass();
//					set_goalpass_07();
//					pl_hosu_set();
//					pl_hosu_make();
//					make_pass_saitan();
//					pass_assyuku();
//					move_sinsaitan();
//
//					Load_Wall();
//					break;
//				}
//			}
		}
		if(G_mode==26){
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					setVmax_Vcom(2800.0, 1000.0, 850.0, 15000.0, 10000.0);
//					G_saitan_mode=1;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					reset_goalpass();
//					set_goalpass_07();
//					pl_hosu_set();
//					pl_hosu_make();
//					make_pass_saitan();
//					pass_assyuku();
//					move_sinsaitan();
//
//					Load_Wall();
//					break;
//				}
//			}
		}
		if(G_mode==27){
//			while(1){
//				if(g_sensor[0][0]>=300&&g_sensor[3][0]>=300){
//					irumi_LED();
//					mpu6500_do_off();
//					Save_Wall();
//					setVmax_Vcom(3000.0, 1000.0, 850.0, 15000.0, 15000.0);
//					G_saitan_mode=1;
//
//					G_look_position_y=0;
//					G_look_position_x=0;
//					G_direction=1080;
//					trim_map_saitan();
//					reset_goalpass();
//					set_goalpass_07();
//					pl_hosu_set();
//					pl_hosu_make();
//					make_pass_saitan();
//					pass_assyuku();
//					move_sinsaitan();
//					Load_Wall();
//					break;
//				}
//			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 10;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
