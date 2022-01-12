/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
std_union g_st_union_1;
std_union g_st_union_2;
	
uint8_t						 			*UART_TX_BUFF;
uint8_t						 			*UART_RX_BUFF;
uint8_t								 	*I2C_TX_BUFF;
uint8_t								 	*I2C_RX_BUFF;
//uint8_t 								dev_address = 0x2C;
uint8_t 								slider_switching_approval = 0x01;
uint8_t 								connected_slider = SLIDER_TWO;
uint8_t 								slider_spec_approval = 0x00;
uint8_t 								AD7147_calibration_approval_1 = 0x01;
uint8_t 								AD7147_calibration_approval_2 = 0x01;
uint8_t 								uart_command;
uint8_t									UART_IRQ_counter = 0;
uint8_t 								UART_first_byte = 0;
uint8_t 								UART_length = 0;
uint8_t									touch_sensor_ctrl = 0;

uint16_t 								active_diode = 0;
uint16_t 								slider_1_AFE_CAL[6];
uint16_t 								slider_2_AFE_CAL[6];

I2C_HandleTypeDef 			hi2c1;
UART_HandleTypeDef 			huart5;

/************************** Variables for AD7147 processing function ***************************/

WORD										AD7147Registers_1[NUMBER_OF_AD7147_REGISTERS];
WORD										AD7147Registers_2[NUMBER_OF_AD7147_REGISTERS];
WORD 										g_SliderStatus_1;
WORD 										g_SliderStatus_2;
BYTE    								InterruptCounterForInitialisation_1;
BYTE   									InterruptCounterForInitialisation_2;
BYTE  									InterruptCounterForThresIntMode_1;
BYTE    								InterruptCounterForThresIntMode_2;
WORD  									SliderStatus_1;
WORD  									SliderStatus_2;
WORD  									AmbientValues_1[NB_OF_SENSORS_FOR_SLIDER];
WORD  									AmbientValues_2[NB_OF_SENSORS_FOR_SLIDER];
WORD  									SensorValues_1[NB_OF_SENSORS_FOR_SLIDER];
WORD  									SensorValues_2[NB_OF_SENSORS_FOR_SLIDER];

//Variables for activation
BYTE  									FirstTimeSliderTouched_1;
WORD  									SliderTouchDownCounter_1;
uint8_t 								SliderFlag_1;
BYTE  									FirstTimeSliderTouched_2;
WORD  									SliderTouchDownCounter_2;
uint8_t 								SliderFlag_2;


//Variables for tapping on the Slider
BYTE  									ReturnTappingCounterOnSlider_1;
BYTE  									NoTouchCounterOnSlider_1;
WORD  									ListBoxUpdateCounterValue_1;
WORD 										ListBoxUpdateCounter_1;
uint8_t 								EnableTapDisplayOnSlider_1;
uint8_t 								TappingFlag_1;
BYTE  									ReturnTappingCounterOnSlider_2;
BYTE  									NoTouchCounterOnSlider_2;
WORD  									ListBoxUpdateCounterValue_2;
WORD  									ListBoxUpdateCounter_2;
uint8_t 								EnableTapDisplayOnSlider_2;
uint8_t 								TappingFlag_2;

BYTE  									NumberOfUpdates_1;
BYTE  									SensorWithHighestValue_1;
BYTE  									NumberOfUpdates_2;
BYTE  									SensorWithHighestValue_2;

//Position variables	
WORD 										SliderPosition_1;
WORD  									AveragePosition_1;
WORD 										SliderPosition_2;
WORD  									AveragePosition_2;

//Relative position variables
uint8_t 								MovedSinceActivation_1;
uint8_t 								PositionOnFirstTouchRecorded_1;
uint8_t 								FastScrollDirection_1;
uint8_t 								RecordSliderCounterOnce_1;
uint8_t 								FastScrollDetected_1;
BYTE  									MinimalNumberOfInterruptsAfterLiftingOff_1;
BYTE  									PositionOnFirstTouch_1;
BYTE  									FastScrollUpdateCounter_1;
WORD  									DisplayItemsResolution_1;
WORD  									PositionOnActivation_1;
uint8_t 								MovedSinceActivation_2;
uint8_t 								PositionOnFirstTouchRecorded_2;
uint8_t 								FastScrollDirection_2;
uint8_t 								RecordSliderCounterOnce_2;
uint8_t 								FastScrollDetected_2;
BYTE  									MinimalNumberOfInterruptsAfterLiftingOff_2;
BYTE  									PositionOnFirstTouch_2;
BYTE  									FastScrollUpdateCounter_2;
WORD  									DisplayItemsResolution_2;
WORD  									PositionOnActivation_2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART5_Init(void);
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

  /* MCU Configuration--------------------------------------------------------*/  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	UART_TX_BUFF = (uint8_t *)malloc(2);
	UART_RX_BUFF = (uint8_t *)malloc(2);
  /* USER CODE END 2 */

	HAL_UART_Transmit(&huart5, ts_read_devid(AD7147_1_ADDRESS), 2, 1);
	HAL_UART_Transmit(&huart5, ts_read_devid(AD7147_2_ADDRESS), 2, 1);
	ConfigAD7147();
	set_lamps_uplight(RED);

	IV6_control_routine(1,22,16, GREEN);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

		if ((touch_sensor_ctrl == 1) && (!(HAL_GPIO_ReadPin(AD7147_IT_PORT_1, AD7147_IT_PIN_1))))
		{
			if(AD7147_calibration_approval_1 == 0x01)
			{
				AD7147_OFFSET_CAL(AD7147_1_ADDRESS);
				AD7147_calibration_approval_1 = 0;
			}
			else if(AD7147_calibration_approval_1 == 0x00)
			{
				ServiceAD7147Isr(AD7147_1_ADDRESS);
				g_st_union_1.istd = g_SliderStatus_1;	
				AD7147_write_to_reg(ATMega_DRIVE, LED_DRIVER(g_st_union_1.istd, AD7147_1_ADDRESS), ATMega1284P_ADDRESS);
				/*UART_TX_BUFF[1] = g_st_union_1.cstd[0];
				UART_TX_BUFF[0] = g_st_union_1.cstd[1];
				HAL_UART_Transmit(&huart5, UART_TX_BUFF, 2, 1);*/
			}
		}
		else if((touch_sensor_ctrl == 1) && (!(HAL_GPIO_ReadPin(AD7147_IT_PORT_2, AD7147_IT_PIN_2))))
		{
			if(AD7147_calibration_approval_2 == 0x01)
			{
				AD7147_OFFSET_CAL(AD7147_2_ADDRESS);
				AD7147_calibration_approval_2 = 0;
			}
			else if(AD7147_calibration_approval_2 == 0x00)
			{
				ServiceAD7147Isr(AD7147_2_ADDRESS);
				g_st_union_2.istd = g_SliderStatus_2;	
				AD7147_write_to_reg(ATMega_DRIVE, LED_DRIVER(g_st_union_2.istd, AD7147_2_ADDRESS), ATMega1284P_ADDRESS);
				/*UART_TX_BUFF[1] = g_st_union_2.cstd[0];
				UART_TX_BUFF[0] = g_st_union_2.cstd[1];
				HAL_UART_Transmit(&huart5, UART_TX_BUFF, 2, 1);*/
			}		
		}
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 300000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 94;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_2;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_TEST_Pin */
  GPIO_InitStruct.Pin = LED_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_1_Pin */
  GPIO_InitStruct.Pin = AD7147_IT_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD7147_IT_PORT_1, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_2_Pin */
  GPIO_InitStruct.Pin = AD7147_IT_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD7147_IT_PORT_2, &GPIO_InitStruct);
	
	/*Configure GPIO pin : LED_0/1_Pin */
  GPIO_InitStruct.Pin = LED_0_Pin | LED_1_Pin | IV6_SHIFT_3_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_0_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pin : SHIFT and IV-6 Pins */
  GPIO_InitStruct.Pin = IV_6_SUPPLY_EN_Pin | IV6_1_NAKAL_CTRL_Pin | IV6_2_NAKAL_CTRL_Pin | IV6_3_NAKAL_CTRL_Pin | IV6_SHIFT_1_DATA_Pin | IV6_SHIFT_2_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IV_6_SUPPLY_EN_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pin : SHIFT and IV-6 Pins */
  GPIO_InitStruct.Pin = IV6_SHIFT_1_MR_Pin | IV6_SHIFT_2_MR_Pin | IV6_SHIFT_3_MR_Pin | IV6_SHIFT_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IV_6_SUPPLY_EN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UART_pack_parser(void)
{
	//uint16_t slider_st;
	std_union temp;
	temp.istd = 0;
	if(UART_TX_BUFF[2] == UART_ADDR)
	{
		if((xor_handler(UART_TX_BUFF) == 0))
		{
			switch(UART_TX_BUFF[3])
			{ 
				case ECHO:
					HAL_UART_Transmit(&huart5, UART_TX_BUFF, UART_TX_BUFF[1], 1);
					uart_command = 0;
				break;	
				case READ_AD7147_DEVID:
					HAL_UART_Transmit(&huart5, ts_read_devid(AD7147_1_ADDRESS), 2, 1);	
					HAL_UART_Transmit(&huart5, ts_read_devid(AD7147_2_ADDRESS), 2, 1);	
					uart_command = 0x01;
				break;
				case READ_ATMega1284P_ID:
					HAL_UART_Transmit(&huart5, ts_read_devid(ATMega1284P_ADDRESS), 2, 1);	
					uart_command = 0x02;
				break;
				case LED_LINE_DRIVE:
					temp.istd = UART_TX_BUFF[4];
					//temp.istd = g_SliderStatus;
					//temp.istd = 2;
					AD7147_write_to_reg(0x0018, temp.istd, ATMega1284P_ADDRESS);
					uart_command = 0x03;
				break;
				case TOUCH_SENS_CTRL:
					touch_sensor_ctrl = UART_TX_BUFF[4];
					uart_command = 0x04;
				break;
				/*case GET_ADC_RES:
 					union _ADC_data{
						uint32_t liadc;
						uint16_t iadc[2];
						uint8_t  cadc[4];
					}ADC_data;

					uint8_t Tr_massive[4];	
					HAL_ADC_Start(&ADC_HandleStruct);
									
					HAL_ADC_PollForConversion(&ADC_HandleStruct, 100);
					ADC_data.liadc = HAL_ADC_GetValue(&ADC_HandleStruct);
					Tr_massive[3] = ADC_data.cadc[0];
					Tr_massive[2] = ADC_data.cadc[1];
					Tr_massive[1] = ADC_data.cadc[2];
					Tr_massive[0] = ADC_data.cadc[3];
					HAL_UART_Transmit(&UART_InitStruct, Tr_massive, 4, 1);	
					HAL_ADC_PollForConversion(&ADC_HandleStruct, 100);
					ADC_data.liadc = HAL_ADC_GetValue(&ADC_HandleStruct);
					Tr_massive[3] = ADC_data.cadc[0];
					Tr_massive[2] = ADC_data.cadc[1];
					Tr_massive[1] = ADC_data.cadc[2];
					Tr_massive[0] = ADC_data.cadc[3];
					HAL_UART_Transmit(&UART_InitStruct, Tr_massive, 4, 1);	
					HAL_ADC_Stop(&ADC_HandleStruct);			
				break;
				case GET_SCALE_RESULT:
					union{
						uint32_t liadc;
						uint16_t iadc[2];
						uint8_t  cadc[4];
					}Scale_data;
					free(UART_TX_BUFF);
					UART_TX_BUFF = (uint8_t *) malloc(4);
					//scale_parser(scale_proc());
					Scale_data.liadc = scale_proc();																//Give the "pure" scale data for calibraion
					UART_TX_BUFF[3] = Scale_data.cadc[0];
					UART_TX_BUFF[2] = Scale_data.cadc[1];
					UART_TX_BUFF[1] = Scale_data.cadc[2];
					UART_TX_BUFF[0] = Scale_data.cadc[3];
					HAL_UART_Transmit(&UART_InitStruct, UART_TX_BUFF, 4, 1);
				break;
				case IV6_ON:
						IV6_control_routine(IV6_number[0], IV6_number[5], IV6_number[8], 1);
				break;*/
				default:
					
				break;
			}	
		}
	}
}

uint8_t xor_handler(uint8_t *mass)
{
	uint8_t result;
	for(int i = 0; i < mass[1]; i++)
	{
		result ^= mass[i];
	}
	return result;
}

uint16_t LED_DRIVER(uint16_t value, uint8_t dev_address)
{
	double data = 0;
	if(value & 0x8000)
	{
		value &= 0x03FF;
		data = value;
		data *= 100;  		//multiply by 100 to get percent
		data /= 0x03FF;
		if(data > 0)
		{
			if(dev_address == AD7147_1_ADDRESS)
			{
				if(data <= 8.33) 
				{
					active_diode = 1;
				}
				else if(data <= 14)
				{
					active_diode = 2;
				}
				else if(data <= 23)
				{
					active_diode = 3;
				}
				else if(data <= 26) //33.33
				{
					active_diode = 4;
				}
				else if(data <= 46) //41.7
				{
					active_diode = 5;
				}
				else if(data <= 49)
				{
					active_diode = 6;
				}
				else if(data <= 58)
				{
					active_diode = 7;
				}
				else if(data <= 66)
				{
					active_diode = 8;
				}
				else if(data <= 75)
				{
					active_diode = 9;
				}
				else if(data <= 83)
				{
					active_diode = 10;
				}
				else if(data <= 88)
				{
					active_diode = 11;
				}
				else if(data <= 100)
				{
					active_diode = 12;
				}
				else 
				{
					active_diode = 0xAA;
				}
				active_diode--;
				}
			else if(dev_address == AD7147_2_ADDRESS)
			{
					if(data <= 8.33) 
				{
					active_diode = 1;
				}
				else if(data <= 16.7)
				{
					active_diode = 2;
				}
				else if(data <= 25)
				{
					active_diode = 3;
				}
				else if(data <= 33.33) //33.33
				{
					active_diode = 4;
				}
				else if(data <= 41.7) //41.7
				{
					active_diode = 5;
				}
				else if(data <= 50)
				{
					active_diode = 6;
				}
				else if(data <= 58.3)
				{
					active_diode = 7;
				}
				else if(data <= 66.7)
				{
					active_diode = 8;
				}
				else if(data <= 75)
				{
					active_diode = 9;
				}
				else if(data <= 83.33)
				{
					active_diode = 10;
				}
				else if(data <= 91.7)
				{
					active_diode = 11;
				}
				else if(data <= 100)
				{
					active_diode = 12;
				}
				else 
				{
					active_diode = 0xAA;
				}
				active_diode += 12;
				active_diode--;
			}
		}
		else
		{
			active_diode = 0xAA;
		}
	}
	else
	{
		active_diode = 0xAA;
	}
	return active_diode;
}

void set_lamps_uplight(uint16_t colour)
{
	HAL_GPIO_WritePin(IV_6_SUPPLY_EN_GPIO_Port, IV_6_SUPPLY_EN_Pin, GPIO_PIN_SET);
	
	if(colour == RED)
	{
		HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
	}
	else if(colour == BLUE)
	{
		HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);		
	}
	else if(colour == GREEN)
	{
		HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);		
	}
	else
	{
		HAL_GPIO_WritePin(LED_0_GPIO_Port, LED_0_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);		
	}
}

void IV6_control_routine(uint8_t lamp1_state, uint8_t lamp2_state, uint8_t lamp3_state, uint8_t diode_colour)
{
	GPIO_PinState pin_local = GPIO_PIN_SET;
	
	HAL_GPIO_WritePin(IV6_1_NAKAL_CTRL_GPIO_Port, IV6_1_NAKAL_CTRL_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(IV6_2_NAKAL_CTRL_GPIO_Port, IV6_2_NAKAL_CTRL_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(IV6_3_NAKAL_CTRL_GPIO_Port, IV6_3_NAKAL_CTRL_Pin, GPIO_PIN_SET);	
	
	HAL_GPIO_WritePin(IV6_SHIFT_1_MR_GPIO_Port, IV6_SHIFT_1_MR_Pin, GPIO_PIN_SET);				//INIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_MR_GPIO_Port, IV6_SHIFT_2_MR_Pin, GPIO_PIN_SET);				//INIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_MR_GPIO_Port, IV6_SHIFT_3_MR_Pin, GPIO_PIN_SET);				//INIT

	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);
	
	if((lamp1_state & 0x80) >> 7)
	{
		pin_local = GPIO_PIN_SET;
		la
	}
	else
	{
		pin_local = GPIO_PIN_RESET;
	}
	
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, pin_local);				//7BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, pin_local);				//7BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, pin_local);				//7BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !((lamp1_state & 0x40) >> 6));			//6BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !((lamp2_state & 0x40) >> 6));			//6BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !((lamp3_state & 0x40) >> 6));			//6BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !((lamp1_state & 0x20) >> 5));				//5BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !((lamp2_state & 0x20) >> 5));				//5BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !((lamp3_state & 0x20) >> 5));				//5BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !((lamp1_state & 0x10) >> 4));								//4BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !((lamp2_state & 0x10) >> 4));								//4BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !((lamp3_state & 0x10) >> 4));								//4BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !((lamp1_state & 0x08) >> 3));								//3BIT	
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !((lamp2_state & 0x08) >> 3));								//3BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !((lamp3_state & 0x08) >> 3));								//3BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !((lamp1_state & 0x04) >> 2));				//2BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !((lamp2_state & 0x04) >> 2));				//2BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !((lamp3_state & 0x04) >> 2));				//2BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !((lamp1_state & 0x02) >> 1));				//1BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !((lamp2_state & 0x02) >> 1));				//1BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !((lamp3_state & 0x02) >> 1));				//1BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, !(lamp1_state & 0x01));			//0BIT
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, !(lamp2_state & 0x01));			//0BIT
	HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, !(lamp3_state & 0x01));			//0BIT
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_SET);
	delay_us(100);
	HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_CLK_Pin, GPIO_PIN_RESET);

	//HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_1_DATA_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(IV6_SHIFT_2_DATA_GPIO_Port, IV6_SHIFT_2_DATA_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(IV6_SHIFT_3_DATA_GPIO_Port, IV6_SHIFT_3_DATA_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
