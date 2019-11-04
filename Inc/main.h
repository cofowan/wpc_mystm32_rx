/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define OK_BIT_0 			( 1 << 0 )
#define BLE_DIS_BIT_1 		( 1 << 1 )
#define OVER_80V_BIT_2 		( 1 << 2 )
#define OVER_FROM_TX_BIT_3 	( 1 << 3 )
#define OVER_LOAD_RX_BIT_4 	( 1 << 4 )

#define LED_TRUN_ON 	( ( GPIO_PinState ) 0 )
#define LED_TRUN_OFF 	( ( GPIO_PinState ) 1 )
typedef struct
{
	uint16_t voltage;
	uint16_t current;
	uint8_t over_load_rx;
	uint8_t keep_live;
}wireless_charge_data_t;

typedef enum
{
		LED_NULL,
		LED_ON_ALWAYS,
		LED_1TIMS_BLINKY,
		LED_2TIMS_BLINKY,
		LED_3TIMS_BLINKY
}led_mode;
typedef enum
{
	NO_KEEP,
	AT_LEAST_KEEP_5S
}keep_mode;
typedef struct{
		void * gpioport;
		uint16_t pin;
		led_mode mode;
		//keep_mode keep;
}led_blinky_t;

#define RES_DWON 	( 30.0f )
#define RES_UP 		( 1000.0f )
#define RES_RATE	( ( RES_DWON / ( RES_DWON + RES_UP ) ) )
#define REF_VOL		( 3.30f )
#define ADC_REF		( ( ( RES_RATE * 4096 ) / REF_VOL ) )

#define DEBUG_ENABLE  0

typedef enum
{
	CUR_NULL,
	CUR_LIGHT_LOAD,
	CUR_UNDER_4A,
	CUR_UNDER_5A,
	CUR_ERROR
}CURRENT_STATUS;
typedef enum
{
	VOL_NULL,
	VOL_UNDER_43V,
	VOL_43V_46V,
	VOL_47V_61V,
	VOL_62V_68V,
	VOL_69V_90V,
	VOL_ABOVE_90V,
	VOL_ERROR
}VOLTAGE_STATUS;
typedef struct 
{

	uint8_t overload 	: 1;
	uint8_t bleonline 	: 1;
	uint8_t voltage		: 3;
	uint8_t current		: 3;
	
}SignalData;

typedef struct
{
	uint16_t vol;
	uint16_t cur;
}data_t;
typedef struct
{
	data_t data;
	uint16_t crc;
	char	 ch;
}vol_cur_t;
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
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOA
#define OVER_LOAD_INT_Pin GPIO_PIN_3
#define OVER_LOAD_INT_GPIO_Port GPIOA
#define OVER_LOAD_INT_EXTI_IRQn EXTI3_IRQn
#define LED_TEST_Pin GPIO_PIN_7
#define LED_TEST_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
