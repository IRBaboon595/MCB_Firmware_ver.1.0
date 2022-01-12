/************************************************
*Author: Alexander Plotnikov
*Data: 09.2017
*Name: AD7147_Lib.h
*Type: AD7147 maintaining heading file
*************************************************/

#include "stdint.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc_ex.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_tim_ex.h"
#include "AD7147regMap.h"

/********** USER DEFs *****************/

/********** FROM AD LIB ***************/

#define AD7147_ADD		0x2C


//---------------------------------
//General definitions
//---------------------------------
#define	BYTE	unsigned char
#define WORD	unsigned int
#define DWORD	unsigned long int

#define	ADUC841			0
#define	AD7147			1

#define SLIDER_ONE 		1
#define SLIDER_TWO 		2

#define COMPLETE 			1
#define THRESHOLD 		2

#define REGISTER_LENGTH			16

#define OFFSET_HIGH_LIMIT		0x8200
#define OFFSET_LOW_LIMIT		0x7D00

//For the initialisation after power up
#define NB_OF_INT				3

//For the memory map of the ADuC841 registers
#define STAGE_START_ADDRESS		10
#define NB_OF_REGS_PER_STAGE	36
#define NB_OF_CONN_REGS			8

#define NUMBER_OF_AD7147_REGISTERS							23	//[0...23 inc]=1st set of registers [0...23 inc]

//---------------------------------
//Definitions
//---------------------------------
//POWER_UP_INTERRUPT: Stage 0 to 11 only. A low limit interrupt on these bits will force calibration
#define POWER_UP_INTERRUPT											0x0FFF
#define STAGE_HIGH_INT_EN_VALUE									0x0FFF
#define NUMBER_OF_INTS_BEFORE_THRES_INT_MODE		18

//"NB_OF_SENSORS_FOR_SLIDER"
//Slider is connected from Stage 0 to Stage 11
#define NB_OF_SENSORS_FOR_SLIDER								12

//"STAGES_USED_FOR_SLIDER"
//This constant represents all the possible stages
//that can be activated touching the slider
#define STAGES_USED_FOR_SLIDER									0x0FFF

#define LOWER_SENSOR_STAGE											0x0001
#define NUMBER_OF_SENSORS_ON_FOR_BIG_FINGER			3

//Definitions for the tap
#define T_MIN_TOUCHING								2	//2*9ms = 18ms
#define T_MAX_TOUCHING								40	//40*9ms = 360ms
#define T_MIN_NO_TOUCHING							20 	//20*9ms = 180ms
#define DISPLAY_AFTER_A_TAP							12	//12*9ms = 108ms

//"MAX_TOUCHDOWN_COUNTER_TIME"
//We stop counting after this amount of Time touching
// on the Slider or Buttons
//255*9ms = 3seconds
#define MAX_TOUCHDOWN_COUNTER_TIME					400 

//"MAX_NO_TOUCH_COUNTER"
//This definition is for stopping our NoTouchCounterOnSlider after a certain number of interrupts
//This should be changed depending on Interrupt Timeout. This is based on 9ms.
 #define MAX_NO_TOUCH_COUNTER						104	//104*9ms = 936ms


//"MIN_NUMBER_OF_UPDATES"
//This definition is used to set the mimimum number of updates 
//when a quick/small swipe is registered
#define MIN_NUMBER_OF_UPDATES						9


//"DISPLAY_ITEMS_CONSTANT"
//DISPLAY_ITEMS_CONSTANT determines how many items 
//can be selected in one scroll
#define DISPLAY_ITEMS_CONSTANT						12

//"LISTBOX_SLOW_UPDATE"
//LISTBOX_SLOW_UPDATE determines the number of interrupts 
//required before starting to autoscroll
#define LISTBOX_SLOW_UPDATE							16

//"LISTBOX_QUICK_UPDATE" determines the number of interrupts 
//required before the next update when autoscrolling
#define LISTBOX_QUICK_UPDATE						4

//"Up/Down" definitions
#define UP											1
#define DOWN										0

//"DISPLACEMENT_OFFSET"
//This is applied to the Unfilterd Slider response to 
//bring smallest value to zero.
#define DISPLACEMENT_OFFSET							3

//"NUMBER_OF_POSITIONS"
//Maximal position available on the Slider.
#define NUMBER_OF_POSITIONS							1023

//"PIXEL_RESOLUTION"
#define PIXEL_RESOLUTION							93

/********* BANK 1 *********************/
/*#define PWR_CONTROL 0x000
#define STAGE_CAL_EN 0x001
#define AMB_COMP_CTRL0 0x002
#define AMB_COMP_CTRL1 0x003
#define AMB_COMP_CTRL2 0x004
#define STAGE_LOW_INT_EN 0x005
#define STAGE_HIGH_INT_EN 0x006
#define STAGE_COMPLETE_INT_EN 0x007
#define STAGE_LOW_INT_ST 0x008
#define STAGE_HIGH_INT_ST 0x009
#define STAGE_COMPLETE_INT_ST 0x00A
#define CDC_RESULT_S0 0x00B
#define DEVID_ID	0x017
#define PROXIM_ST	0x042*/

/********* BANK 2 *********************/
/*#define STAGE_0_CONNECTION_1 0x080
#define STAGE_0_CONNECTION_2 0x081
#define STAGE_0_AFE_OFFSET 0x082
#define STAGE_0_SENSIVITY 0x083
#define STAGE_0_OFFSET_LOW 0x084
#define STAGE_0_OFFSET_HIGH 0x085
#define STAGE_0_OFFSET_HIGH_CLAMP 0x086
#define STAGE_0_OFFSET_LOW_CLAMP 0x087*/


extern WORD											AD7147Registers_1[NUMBER_OF_AD7147_REGISTERS];
extern WORD											AD7147Registers_2[NUMBER_OF_AD7147_REGISTERS];
extern WORD 										g_SliderStatus_1;
extern WORD 										g_SliderStatus_2;
extern BYTE    									InterruptCounterForInitialisation_1;
extern BYTE   									InterruptCounterForInitialisation_2;
extern BYTE  									  InterruptCounterForThresIntMode_1;
extern BYTE    									InterruptCounterForThresIntMode_2;
extern WORD   									SliderStatus_1;
extern WORD   									SliderStatus_2;
extern WORD   									AmbientValues_1[NB_OF_SENSORS_FOR_SLIDER];
extern WORD    									AmbientValues_2[NB_OF_SENSORS_FOR_SLIDER];
extern WORD    									SensorValues_1[NB_OF_SENSORS_FOR_SLIDER];
extern WORD    									SensorValues_2[NB_OF_SENSORS_FOR_SLIDER];

//Variables for activation
extern BYTE  										FirstTimeSliderTouched_1;
extern WORD  										SliderTouchDownCounter_1;
extern uint8_t 									SliderFlag_1;
extern BYTE  										FirstTimeSliderTouched_2;
extern WORD  										SliderTouchDownCounter_2;
extern uint8_t 									SliderFlag_2;

//Variables for tapping on the Slider
extern BYTE    									ReturnTappingCounterOnSlider_1;
extern BYTE    									NoTouchCounterOnSlider_1;
extern WORD    									ListBoxUpdateCounterValue_1;
extern WORD    									ListBoxUpdateCounter_1;
extern uint8_t 									EnableTapDisplayOnSlider_1;
extern uint8_t 									TappingFlag_1;
extern BYTE    									ReturnTappingCounterOnSlider_2;
extern BYTE    									NoTouchCounterOnSlider_2;
extern WORD    									ListBoxUpdateCounterValue_2;
extern WORD   									ListBoxUpdateCounter_2;
extern uint8_t						 			EnableTapDisplayOnSlider_2;
extern uint8_t 									TappingFlag_2;

extern BYTE  										NumberOfUpdates_1;
extern BYTE  										SensorWithHighestValue_1;
extern BYTE  										NumberOfUpdates_2;
extern BYTE  										SensorWithHighestValue_2;

//Position variables	
extern WORD 										SliderPosition_1;
extern WORD  										AveragePosition_1;
extern WORD 										SliderPosition_2;
extern WORD  										AveragePosition_2;

//Relative position variables
extern uint8_t 								MovedSinceActivation_1;
extern uint8_t 								PositionOnFirstTouchRecorded_1;
extern uint8_t 								FastScrollDirection_1;
extern uint8_t 								RecordSliderCounterOnce_1;
extern uint8_t 								FastScrollDetected_1;
extern BYTE  									MinimalNumberOfInterruptsAfterLiftingOff_1;
extern BYTE  									PositionOnFirstTouch_1;
extern BYTE  									FastScrollUpdateCounter_1;
extern WORD  									DisplayItemsResolution_1;
extern WORD  									PositionOnActivation_1;
extern uint8_t 								MovedSinceActivation_2;
extern uint8_t 								PositionOnFirstTouchRecorded_2;
extern uint8_t 								FastScrollDirection_2;
extern uint8_t 								RecordSliderCounterOnce_2;
extern uint8_t 								FastScrollDetected_2;
extern BYTE  									MinimalNumberOfInterruptsAfterLiftingOff_2;
extern BYTE  									PositionOnFirstTouch_2;
extern BYTE  									FastScrollUpdateCounter_2;
extern WORD  									DisplayItemsResolution_2;
extern WORD  									PositionOnActivation_2;

uint8_t* ts_read_devid(uint8_t dev_address);
void AD7147_write_to_reg(uint16_t reg_address, uint16_t reg_data, uint8_t dev_address);
uint8_t* AD7147_read_from_reg(uint16_t reg_address, uint8_t dev_address);
void AD7147_POWERUP_SEQ(void);
void AD7147_POWER_DOWN(void);
void WriteToAD7147(WORD RegisterAddress, BYTE NumberOfRegisters, WORD *DataBuffer, BYTE OffsetInBuffer, uint8_t dev_address);
void ReadFromAD7147(WORD RegisterStartAddress, BYTE NumberOfRegisters, WORD *DataBuffer, WORD OffsetInBuffer, uint8_t dev_address);
void ServiceAD7147Isr(uint8_t dev_address);
void ForceCalibration(uint8_t dev_address);
void ConfigAD7147(void);
void InitialiseSlider(uint8_t dev_address);
WORD GetNewSliderUpdate(uint8_t dev_address, WORD *reg_container);
BYTE FindHighestAndLowestStagesUsed(WORD InterruptStatusRegister, WORD LowestStageOfTheSensor, BYTE NumberOfStagesUsed, BYTE BigFingerLevel);
void AD7147_OFFSET_CAL(uint8_t dev_address);
