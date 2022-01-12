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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h" /* memset */
#include "AD7147_Lib.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t						 		*I2C_TX_BUFF;
extern uint8_t						 		*I2C_RX_BUFF;
extern uint8_t						 		*UART_TX_BUFF;
extern uint8_t						 		*UART_RX_BUFF;
//extern uint8_t 								dev_address;
extern uint8_t								slider_switching_approval;
extern uint8_t 								connected_slider;
extern uint8_t 								slider_spec_approval;
extern uint8_t 								AD7147_calibration_approval_1;
extern uint8_t 								AD7147_calibration_approval_2;
extern uint8_t 								uart_command;
extern uint8_t								UART_IRQ_counter;
extern uint8_t 								UART_first_byte;
extern uint8_t 								UART_length;
extern uint8_t								touch_sensor_ctrl;

extern uint16_t 								active_diode;
extern uint16_t 							slider_1_AFE_CAL[6];
extern uint16_t 							slider_2_AFE_CAL[6];
	
	
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart5;	

/*typedef struct _tim_struct
{
	TIM_HandleTypeDef Timer_InitStruct_2;
	TIM_Base_InitTypeDef Timer_BaseParam_2;
	TIM_OC_InitTypeDef TimOCHandle_2;

	TIM_HandleTypeDef Timer_InitStruct_3;
	TIM_Base_InitTypeDef Timer_BaseParam_3;
	TIM_OC_InitTypeDef TimOCHandle_3;

	TIM_HandleTypeDef Timer_InitStruct_4;
	TIM_Base_InitTypeDef Timer_BaseParam_4;
	TIM_OC_InitTypeDef TimOCHandle_4;
}tim_struct;*/

typedef union 
{
	uint16_t istd;
	uint8_t  cstd[2];
}std_union;
	
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

void UART_pack_parser(void);
uint8_t xor_handler(uint8_t *mass);
uint16_t LED_DRIVER(uint16_t value, uint8_t dev_address);
void set_lamps_uplight(uint16_t colour);
void IV6_control_routine(uint8_t lamp1_state, uint8_t lamp2_state, uint8_t lamp3_state, uint8_t diode_colour);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_TEST_Pin 								GPIO_PIN_9
#define LED_TEST_GPIO_Port 					GPIOC
#define SCL_Pin 										GPIO_PIN_6
#define SCL_GPIO_Port 							GPIOB
#define SDA_Pin 										GPIO_PIN_7
#define SDA_GPIO_Port 							GPIOB
#define SENSOR_1_Pin 								GPIO_PIN_8
#define SENSOR_1_GPIO_Port 					GPIOB
#define SENSOR_2_Pin 								GPIO_PIN_1
#define SENSOR_2_GPIO_Port 					GPIOE

#define ON 													0x01
#define OFF 												0x00

/************************************** TOUCH SENSOR USER Defines ***************************************/

#define AD7147_DEVID_REG 						0x017
#define AD7147_1_ADDRESS 						0x2C
#define AD7147_2_ADDRESS 						0x2D
#define AD7147_WRITE 	 							0x00
#define AD7147_READ 	 							0x01
#define AD7147_IT_PORT_2 						GPIOB
#define AD7147_IT_PIN_2 						GPIO_PIN_8
#define AD7147_IT_PORT_1 						GPIOE
#define AD7147_IT_PIN_1							GPIO_PIN_1

#define LED_LINE_PORT_1 						GPIOD
#define LED_LINE_EN_1 							GPIO_PIN_9
#define LED_LINE_EN_2 							GPIO_PIN_12
#define LED_LINE_EN_3 							GPIO_PIN_14

#define LED_LINE_PORT_2 						GPIOE
#define LED_LINE_CLK_3 							GPIO_PIN_4
#define LED_LINE_CLK_2 							GPIO_PIN_2
#define LED_LINE_CLK_1 							GPIO_PIN_12
#define LED_LINE_DATA 							GPIO_PIN_15

#define ATMega1284P_ADDRESS 				0x2E
#define ATMega_ID										0x17
#define ATMega_DRIVE								0x18

/************************************** IV-6 USER Defines ***************************************/

#define LED_0_GPIO_Port 						GPIOD
#define LED_0_Pin										GPIO_PIN_9
#define LED_1_GPIO_Port 						GPIOD
#define LED_1_Pin										GPIO_PIN_12

#define IV_6_SUPPLY_EN_GPIO_Port		GPIOE
#define IV_6_SUPPLY_EN_Pin					GPIO_PIN_13
#define IV6_1_NAKAL_CTRL_GPIO_Port	GPIOE
#define IV6_1_NAKAL_CTRL_Pin				GPIO_PIN_7
#define IV6_2_NAKAL_CTRL_GPIO_Port	GPIOE
#define IV6_2_NAKAL_CTRL_Pin				GPIO_PIN_12
#define IV6_3_NAKAL_CTRL_GPIO_Port	GPIOE
#define IV6_3_NAKAL_CTRL_Pin				GPIO_PIN_14
#define IV6_SHIFT_1_DATA_GPIO_Port	GPIOE
#define IV6_SHIFT_1_DATA_Pin				GPIO_PIN_0
#define IV6_SHIFT_2_DATA_GPIO_Port	GPIOE
#define IV6_SHIFT_2_DATA_Pin				GPIO_PIN_5
#define IV6_SHIFT_3_DATA_GPIO_Port	GPIOD
#define IV6_SHIFT_3_DATA_Pin				GPIO_PIN_10
#define IV6_SHIFT_1_MR_GPIO_Port		GPIOE
#define IV6_SHIFT_1_MR_Pin					GPIO_PIN_6
#define IV6_SHIFT_2_MR_GPIO_Port		GPIOE
#define IV6_SHIFT_2_MR_Pin					GPIO_PIN_8
#define IV6_SHIFT_3_MR_GPIO_Port		GPIOE
#define IV6_SHIFT_3_MR_Pin					GPIO_PIN_10
#define IV6_SHIFT_CLK_GPIO_Port			GPIOE
#define IV6_SHIFT_CLK_Pin						GPIO_PIN_9

#define RED													1
#define BLUE												2
#define GREEN												3


/*************************************** UART DEFINES *****************************************************/

#define SYNCHRO											0x02
#define UART_ADDR										0x0A
#define ECHO												0x00
#define READ_AD7147_DEVID						0x01
#define	READ_ATMega1284P_ID					0x02
#define	LED_LINE_DRIVE							0x03
#define	TOUCH_SENS_CTRL							0x04
//......... OTHER FUNCTIONS




/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
