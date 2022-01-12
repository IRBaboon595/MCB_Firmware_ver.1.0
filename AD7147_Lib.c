	/************************************************
	*Author: Alexander Plotnikov
	*Data: 09.2017
	*Name: AD7147_Lib.c
	*Type: AD7147 maintaining routines source file
	*************************************************/

	#include "AD7147_Lib.h"
	#include "main.h"
	
	uint8_t* ts_read_devid(uint8_t dev_address)
	{
		//Massive clearing
		free(I2C_TX_BUFF);
		free(I2C_RX_BUFF);
		I2C_TX_BUFF = (uint8_t *) malloc(2);
		I2C_RX_BUFF = (uint8_t *) malloc(2);
		memset(I2C_RX_BUFF, 0, sizeof(I2C_RX_BUFF));
		memset(I2C_RX_BUFF, 0, sizeof(I2C_TX_BUFF));
		dev_address = (dev_address << 1);
		
		//Address and function declare AD7147_DEVID_REG - also MK's ID register
		I2C_TX_BUFF[0] = ((AD7147_DEVID_REG & 0xFF00) >> 8);
		I2C_TX_BUFF[1] = (AD7147_DEVID_REG & 0x00FF);
		
		HAL_I2C_Master_Transmit(&hi2c1, (dev_address & ~(1 << AD7147_WRITE)), I2C_TX_BUFF, 2, 1);
		HAL_I2C_Master_Receive(&hi2c1, (dev_address | AD7147_READ), I2C_RX_BUFF, 2, 1);
		
		return I2C_RX_BUFF;
	}

	void AD7147_write_to_reg(uint16_t reg_address, uint16_t reg_data, uint8_t dev_address)
	{
		free(I2C_TX_BUFF);
		I2C_TX_BUFF = (uint8_t *) malloc(4);
		memset(I2C_RX_BUFF, 0, sizeof(I2C_TX_BUFF));
		dev_address = (dev_address << 1);
		
		I2C_TX_BUFF[0] = ((reg_address & 0xFF00) >> 8);
		I2C_TX_BUFF[1] = (reg_address & 0x00FF);
		I2C_TX_BUFF[2] = ((reg_data & 0xFF00) >> 8);
		I2C_TX_BUFF[3] = (reg_data & 0x00FF);
		
		HAL_I2C_Master_Transmit(&hi2c1, (dev_address & ~(1 << AD7147_WRITE)), I2C_TX_BUFF, 4, 1);
	}

	uint8_t* AD7147_read_from_reg(uint16_t reg_address, uint8_t dev_address)
	{
		free(I2C_TX_BUFF);
		free(I2C_RX_BUFF);
		I2C_TX_BUFF = (uint8_t *) malloc(2);
		I2C_RX_BUFF = (uint8_t *) malloc(2);
		memset(I2C_RX_BUFF, 0, sizeof(I2C_RX_BUFF));
		memset(I2C_RX_BUFF, 0, sizeof(I2C_TX_BUFF));
		dev_address = (dev_address << 1);
		
		I2C_TX_BUFF[0] = ((reg_address & 0xFF00) >> 8);
		I2C_TX_BUFF[1] = (reg_address & 0x00FF);
		
		HAL_I2C_Master_Transmit(&hi2c1, (dev_address & ~(1 << AD7147_WRITE)), I2C_TX_BUFF, 2, 1);
		HAL_I2C_Master_Receive(&hi2c1, (dev_address | AD7147_READ), I2C_RX_BUFF, 2, 1);

		return I2C_RX_BUFF;
	}

	void AD7147_POWERUP_SEQ(void)
	{
		
		/**************** SETTING BANK 2 REGISTERS ***************************/
		AD7147_write_to_reg((STAGE0_CONNECTION), 0x3FFE, AD7147_1_ADDRESS);
		AD7147_write_to_reg((STAGE0_CONNECTION+1), 0x1FFF, AD7147_1_ADDRESS);
		for(uint16_t i = (STAGE0_CONNECTION+2); i < ((STAGE0_CONNECTION+7) + 1); i++)
		{
			AD7147_write_to_reg(i, 0x0000, AD7147_1_ADDRESS);
		}
		for(uint16_t f = 1; f < 12; f++)
		{
			AD7147_write_to_reg((STAGE0_CONNECTION + 8*f), 0x0000, AD7147_1_ADDRESS);
			AD7147_write_to_reg(((STAGE0_CONNECTION+1) + 8*f), 0xC000, AD7147_1_ADDRESS);
			for(uint16_t i = ((STAGE0_CONNECTION+2) + 8*f); i < (((STAGE0_CONNECTION+7) + 8*f) + 1); i++)
			{
				AD7147_write_to_reg(i, 0x0000, AD7147_1_ADDRESS);
			}
		}
		/**************** SETTING BANK 1 REGISTERS ***************************/
		AD7147_write_to_reg(PWR_CONTROL, 0x1000, AD7147_1_ADDRESS);
		AD7147_write_to_reg(STAGE_CAL_EN, 0x0000, AD7147_1_ADDRESS);
		AD7147_write_to_reg(AMB_COMP_CTRL0, 0x3230, AD7147_1_ADDRESS);
		AD7147_write_to_reg(AMB_COMP_CTRL1, 0x0419, AD7147_1_ADDRESS);
		AD7147_write_to_reg(AMB_COMP_CTRL2, 0x0340, AD7147_1_ADDRESS);
		AD7147_write_to_reg(STAGE_LOW_INT_EN, 0x0000, AD7147_1_ADDRESS);
		AD7147_write_to_reg(STAGE_HIGH_INT_EN, 0x0000, AD7147_1_ADDRESS);
		AD7147_write_to_reg(STAGE_COMPLETE_INT_EN, 0x0001, AD7147_1_ADDRESS);
		
		/**************** ENBALE CONVERSION **************************/
		AD7147_write_to_reg(STAGE_CAL_EN, 0x0001, AD7147_1_ADDRESS);
		//uint8_t temp_s[] = {"You_are_gay"};
		//HAL_UART_Transmit(&huart5, temp_s, 11, 1);
	}

	void AD7147_POWER_DOWN(void)
	{
		AD7147_write_to_reg(PWR_CONTROL, 0x0601, AD7147_1_ADDRESS);
	}

	void ConfigAD7147(void)
	{
		uint8_t *temp_massive;
		temp_massive = (uint8_t*)malloc(2);
		//THIS ROUTINE RECONFIGURED FOR 2 SENSORS. 12 STAGES FOR 12 CINs.
		WORD  StageBuffer[8];
		
		AD7147Registers_1[PWR_CONTROL]=0x0600;	//Register 0x00
		AD7147Registers_2[PWR_CONTROL]=0x0600;
		AD7147_write_to_reg(PWR_CONTROL, AD7147Registers_1[PWR_CONTROL], AD7147_1_ADDRESS);
		AD7147_write_to_reg(PWR_CONTROL, AD7147Registers_2[PWR_CONTROL], AD7147_2_ADDRESS);

		//temp_massive = AD7147_read_from_reg(PWR_CONTROL, AD7147_1_ADDRESS);
		//HAL_UART_Transmit(&huart5, temp_massive, 2, 1);

		//temp_massive = AD7147_read_from_reg(PWR_CONTROL, AD7147_2_ADDRESS);
		//HAL_UART_Transmit(&huart5, temp_massive, 2, 1);

		//======================
		//= Stage 0 CIN0(+) S1 =
		//======================
		StageBuffer[0]=0x3FFE;	//Register 0x80
		StageBuffer[1]=0x1FFF;	//Register 0x81
		StageBuffer[2]=0x8080;	//Register 0x82
		StageBuffer[3]=0x2626;	//Register 0x83
		StageBuffer[4]=2000;	//Register 0x84
		StageBuffer[5]=2000;	//Register 0x85
		StageBuffer[6]=2000;	//Register 0x86
		StageBuffer[7]=2000;	//Register 0x87
		WriteToAD7147(STAGE0_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE0_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		//AD7147_write_to_reg(STAGE0_CONNECTION, 0xFFFE, AD7147_1_ADDRESS);
		//AD7147_write_to_reg(STAGE0_CONNECTION+1, 0x1FFF, AD7147_1_ADDRESS);
		
		for(int i = 0; i < 8; i++)
		{
			temp_massive = AD7147_read_from_reg(0x0080 + i, AD7147_1_ADDRESS);
			HAL_UART_Transmit(&huart5, temp_massive, 2, 1);
		}
	
		//======================
		//= Stage 1 CIN1(+) S2 =
		//======================
		StageBuffer[0]=0x3FFB;	//Register 0x88
		StageBuffer[1]=0x1FFF;	//Register 0x89
		StageBuffer[2]=0x8080;	//Register 0x8A
		StageBuffer[3]=0x2626;	//Register 0x8B
		StageBuffer[4]=2000;	//Register 0x84
		StageBuffer[5]=2000;	//Register 0x85
		StageBuffer[6]=2000;	//Register 0x86
		StageBuffer[7]=2000;	//Register 0x87
		WriteToAD7147(STAGE1_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE1_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		//========================
		//= Stage 2 - CIN2(+) S3 =
		//========================
		StageBuffer[0]=0x3FEF;	//Register 0x90
		StageBuffer[1]=0x1FFF;	//Register 0x91
		StageBuffer[2]=0x8080;	//Register 0x92
		StageBuffer[3]=0x2626;	//Register 0x93
		StageBuffer[4]=2000;	//Register 0x84
		StageBuffer[5]=2000;	//Register 0x85
		StageBuffer[6]=2000;	//Register 0x86
		StageBuffer[7]=2000;	//Register 0x87
		WriteToAD7147(STAGE2_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE2_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);

		//========================
		//= Stage 3 - CIN3(+) S4 =
		//========================
		StageBuffer[0]=0x3FBF;	//Register 0x98
		StageBuffer[1]=0x1FFF;	//Register 0x99
		StageBuffer[2]=0x8080;	//Register 0x9A
		StageBuffer[3]=0x2626;	//Register 0x9B
		StageBuffer[4]=2000;	//Register 0x84
		StageBuffer[5]=2000;	//Register 0x85
		StageBuffer[6]=2000;	//Register 0x86
		StageBuffer[7]=2000;	//Register 0x87
		WriteToAD7147(STAGE3_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE3_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);

		//========================
		//= Stage 4 - CIN4(+) S5 = 
		//========================
		StageBuffer[0]=0x3EFF;	//Register 0xA0
		StageBuffer[1]=0x1FFF;	//Register 0xA1
		StageBuffer[2]=0x8080;	//Register 0xA2
		StageBuffer[3]=0x2626;	//Register 0xA3
		StageBuffer[4]=2000;	//Register 0x84
		StageBuffer[5]=2000;	//Register 0x85
		StageBuffer[6]=2000;	//Register 0x86
		StageBuffer[7]=2000;	//Register 0x87
		WriteToAD7147(STAGE4_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE4_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		//=========================
		//= Stage 5 -  CIN5(+) S6 =
		//=========================
		StageBuffer[0]=0x3BFF;	//Register 0xA8
		StageBuffer[1]=0x1FFF;	//Register 0xA9
		StageBuffer[2]=0x8080;	//Register 0xAA
		StageBuffer[3]=0x2626;	//Register 0xAB
		StageBuffer[4]=2000;	//Register 0x84
		StageBuffer[5]=2000;	//Register 0x85
		StageBuffer[6]=2000;	//Register 0x86
		StageBuffer[7]=2000;	//Register 0x87
		WriteToAD7147(STAGE5_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE5_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		//=========================
		//= Stage 6 - CIN6(+) S7 =
		//=========================
		StageBuffer[0]=0x2FFF;	//Register 0xC8
		StageBuffer[1]=0x1FFF;	//Register 0xC9
		StageBuffer[2]=0x8080;	//Register 0xCA
		StageBuffer[3]=0x2626;	//Register 0xCB
		StageBuffer[4]=2000;	//Register 0xCC
		StageBuffer[5]=2000;	//Register 0xCD
		StageBuffer[6]=2000;	//Register 0xCE
		StageBuffer[7]=2000;	//Register 0xCF
		WriteToAD7147(STAGE6_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE6_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		//=========================
		//= Stage 7 - CIN7(+) S8 =
		//=========================
		StageBuffer[0]=0x3FFF;	//Register 0xC8
		StageBuffer[1]=0x1FFE;	//Register 0xC9		//DOUBLE!
		StageBuffer[2]=0x8080;	//Register 0xCA
		StageBuffer[3]=0x2626;	//Register 0xCB
		StageBuffer[4]=2000;	//Register 0xCC
		StageBuffer[5]=2000;	//Register 0xCD
		StageBuffer[6]=2000;	//Register 0xCE
		StageBuffer[7]=2000;	//Register 0xCF
		WriteToAD7147(STAGE7_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE7_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
			
		//====================
		//= Stage 8 - CIN8(+) S9 =
		//====================
		StageBuffer[0]=0x3FFF;	//Register 0xC0
		StageBuffer[1]=0x1FFB;	//Register 0xC1
		StageBuffer[2]=0x8080;	//Register 0xC2
		StageBuffer[3]=0x2626;	//Register 0xC3
		StageBuffer[4]=2000;	//Register 0xC4
		StageBuffer[5]=2000;	//Register 0xC5
		StageBuffer[6]=2000;	//Register 0xC6
		StageBuffer[7]=2000;	//Register 0xC7
		WriteToAD7147(STAGE8_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE8_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		//===================
		// Stage 9 - CIN9(+) S10 =
		//===================
		StageBuffer[0]=0x3FFF;	//Register 0xC8
		StageBuffer[1]=0x1FEF;	//Register 0xC9
		StageBuffer[2]=0x8080;	//Register 0xCA
		StageBuffer[3]=0x2626;	//Register 0xCB
		StageBuffer[4]=2000;	//Register 0xCC
		StageBuffer[5]=2000;	//Register 0xCD
		StageBuffer[6]=2000;	//Register 0xCE
		StageBuffer[7]=2000;	//Register 0xCF
		WriteToAD7147(STAGE9_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE9_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);

		//=====================
		//= Stage 10 - CIN10(+) S11 =
		StageBuffer[0]=0x3FFF;	//Register 0xD0
		StageBuffer[1]=0x1FBF;	//Register 0xD1
		StageBuffer[2]=0x8080;	//Register 0xD2
		StageBuffer[3]=0x2626;	//Register 0xD3
		StageBuffer[4]=2000;	//Register 0xD4
		StageBuffer[5]=2000;	//Register 0xD5
		StageBuffer[6]=2000;	//Register 0xD6
		StageBuffer[7]=2000;	//Register 0xD7
		WriteToAD7147(STAGE10_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE10_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		//=====================
		//= Stage 11 - CIN11(+) S12 =
		//=====================
		StageBuffer[0]=0x3FFF;	//Register 0xD8
		StageBuffer[1]=0x1EFF;	//Register 0xD9
		StageBuffer[2]=0x8080;	//Register 0xDA
		StageBuffer[3]=0x2626;	//Register 0xDB
		StageBuffer[4]=2000;	//Register 0xDC
		StageBuffer[5]=2000;	//Register 0xDD
		StageBuffer[6]=2000;	//Register 0xDE
		StageBuffer[7]=2000;	//Register 0xDF
		WriteToAD7147(STAGE11_CONNECTION, 8, StageBuffer, 0, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE11_CONNECTION, 8, StageBuffer, 0, AD7147_2_ADDRESS);
		
		
		//--------------------------------------------------------------------------//
		//-------------------------Bank 1 Registers---------------------------------//
		//--------------------------------------------------------------------------//
		//Initialisation of the first register bank but not the AMBCOMPCTL_REG0
		AD7147Registers_1[PWR_CONTROL]=0x02B2;	//Register 0x00
		AD7147Registers_2[PWR_CONTROL]=0x02B2;	//Register 0x00
		AD7147_write_to_reg(PWR_CONTROL, AD7147Registers_1[PWR_CONTROL], AD7147_1_ADDRESS);
		AD7147_write_to_reg(PWR_CONTROL, AD7147Registers_2[PWR_CONTROL], AD7147_2_ADDRESS);
		//temp_massive = AD7147_read_from_reg(PWR_CONTROL, AD7147_1_ADDRESS);
		//HAL_UART_Transmit(&huart5, temp_massive, 2, 1);
		ReadFromAD7147(STAGE_LOW_LIMIT_INT, 3, AD7147Registers_1, STAGE_LOW_LIMIT_INT, AD7147_1_ADDRESS);
		ReadFromAD7147(STAGE_LOW_LIMIT_INT, 3, AD7147Registers_2, STAGE_LOW_LIMIT_INT, AD7147_2_ADDRESS);
		AD7147Registers_1[AMB_COMP_CTRL0]=0x3233;	//Register 0x02
		AD7147Registers_1[AMB_COMP_CTRL1]=0x0A19;	//Register 0x03
		AD7147Registers_1[AMB_COMP_CTRL2]=0x0832;	//Register 0x04
		AD7147Registers_1[STAGE_LOW_INT_EN]=0x0000;//Register 0x05
		AD7147Registers_1[STAGE_HIGH_INT_EN]=0x0000;	//Register 0x06
		AD7147Registers_1[STAGE_COMPLETE_INT_EN]=0x0001;	//Register 0x07
		AD7147Registers_2[AMB_COMP_CTRL0]=0x3233;	//Register 0x02
		AD7147Registers_2[AMB_COMP_CTRL1]=0x0A19;	//Register 0x03
		AD7147Registers_2[AMB_COMP_CTRL2]=0x0832;	//Register 0x04
		AD7147Registers_2[STAGE_LOW_INT_EN]=0x0000;//Register 0x05
		AD7147Registers_2[STAGE_HIGH_INT_EN]=0x0000;	//Register 0x06
		AD7147Registers_2[STAGE_COMPLETE_INT_EN]=0x0001;	//Register 0x07
		WriteToAD7147(AMB_COMP_CTRL0, 6, AD7147Registers_1, AMB_COMP_CTRL0, AD7147_1_ADDRESS);
		WriteToAD7147(AMB_COMP_CTRL0, 6, AD7147Registers_2, AMB_COMP_CTRL0, AD7147_2_ADDRESS);
			
		//Enable data path for all sequences
		AD7147Registers_1[STAGE_CAL_EN]=0x0FFF;	//Register 0x01
		AD7147Registers_2[STAGE_CAL_EN]=0x0FFF;	//Register 0x01
		WriteToAD7147(STAGE_CAL_EN, 1, AD7147Registers_1, STAGE_CAL_EN, AD7147_1_ADDRESS);
		WriteToAD7147(STAGE_CAL_EN, 1, AD7147Registers_2, STAGE_CAL_EN, AD7147_2_ADDRESS);
		/*for(int i = 0; i < 8; i++)
		{
			HAL_UART_Transmit(&huart5, AD7147_read_from_reg(PWR_CONTROL + i, AD7147_2_ADDRESS), 2, 1);
		}*/

		ReadFromAD7147(STAGE_LOW_LIMIT_INT, 3, AD7147Registers_1, STAGE_LOW_LIMIT_INT, AD7147_1_ADDRESS);
		ReadFromAD7147(STAGE_LOW_LIMIT_INT, 3, AD7147Registers_2, STAGE_LOW_LIMIT_INT, AD7147_2_ADDRESS);
		HAL_Delay(500);
	}

	//---------------------------------
	//WriteToAD7147();
	//---------------------------------
	//This function writes to the AD7147 either via I2C.
	//--------------------------------------------------------------------------------
	void WriteToAD7147(WORD RegisterAddress, BYTE NumberOfRegisters, WORD *DataBuffer, BYTE OffsetInBuffer, uint8_t dev_address)
	{
		for(uint16_t i = RegisterAddress; i < (RegisterAddress + NumberOfRegisters); i++)
		{
			AD7147_write_to_reg(i, DataBuffer[OffsetInBuffer + (i - RegisterAddress)], dev_address);
		}
		//WriteToAD7147ViaI2C(RegisterAddress, NumberOfRegisters, DataBuffer, OffsetInBuffer);
	}


	//---------------------------------
	//ReadFromAD7147();
	//---------------------------------
	//This function reads from the AD7147 via I2C.
	//--------------------------------------------------------------------------------
	void ReadFromAD7147(WORD RegisterStartAddress, BYTE NumberOfRegisters, WORD *DataBuffer, WORD OffsetInBuffer, uint8_t dev_address)
	{
		std_union temp_1;	
		//uint8_t		add_temp = 0;
		
		for(uint16_t i = RegisterStartAddress; i < (RegisterStartAddress + NumberOfRegisters); i++)
		{		
			AD7147_read_from_reg(i, dev_address);
			temp_1.cstd[0] = I2C_RX_BUFF[1];
			temp_1.cstd[1] = I2C_RX_BUFF[0];
			DataBuffer[OffsetInBuffer + (i - RegisterStartAddress)] = temp_1.istd;
		}
		//ReadFromAD7147ViaI2C(RegisterStartAddress, NumberOfRegisters, DataBuffer, OffsetInBuffer);
	}

	//ServiceAD7147Isr();
	//-----------------------------------------------------------------
	// Function called by the AD7147 ISR. Anything that must be 
	// executed during the ISR needs to be done here
	//-----------------------------------------------------------------
	void ServiceAD7147Isr(uint8_t dev_address)
	{			
		if(dev_address == AD7147_1_ADDRESS)
		{
			//Read thresholds registers
			ReadFromAD7147(STAGE_LOW_LIMIT_INT, 3, AD7147Registers_1, STAGE_LOW_LIMIT_INT, AD7147_1_ADDRESS);	
			if(InterruptCounterForInitialisation_1 < (NB_OF_INT+2))
			{		
				if (InterruptCounterForInitialisation_1==(NB_OF_INT-1))
				{
					//================================
					//= Put initialisation code here =
					//================================
					InitialiseSlider(AD7147_1_ADDRESS);
				}
				InterruptCounterForInitialisation_1++;
			}		
			if(InterruptCounterForInitialisation_1>=NB_OF_INT)
			{
				//============================
				//= Recalibrate if required. =
				//============================
				
				if ((AD7147Registers_1[STAGE_LOW_LIMIT_INT] & POWER_UP_INTERRUPT) != 0x0000)
				{
					ForceCalibration(AD7147_1_ADDRESS);
					InterruptCounterForInitialisation_1 = 0;
				}
				else
				{
					//Get a new slider position status
					g_SliderStatus_1 = GetNewSliderUpdate(AD7147_1_ADDRESS, AD7147Registers_1); //"g_SliderStatus" is updated from this line on.
				}	
				//=======================
				// Change interrupt mode
				//=======================
				//Configure AD7147 in EOC interrupt driven mode
				if(((AD7147Registers_1[STAGE_HIGH_LIMIT_INT] & POWER_UP_INTERRUPT)!=0x0000) || 
					((AD7147Registers_1[STAGE_LOW_LIMIT_INT] & POWER_UP_INTERRUPT)!=0x0000))
				{
					if(AD7147Registers_1[STAGE_COMPLETE_INT_EN]==0x0000)
					{
						AD7147Registers_1[STAGE_LOW_INT_EN] &= 0xF000;
						AD7147Registers_1[STAGE_HIGH_INT_EN] &= 0xF000;
						AD7147Registers_1[STAGE_COMPLETE_INT_EN]=0x0001;
						WriteToAD7147(STAGE_LOW_INT_EN, 3, AD7147Registers_1, STAGE_LOW_INT_EN, AD7147_1_ADDRESS);
						slider_switching_approval = COMPLETE;
					}
					InterruptCounterForThresIntMode_1 = NUMBER_OF_INTS_BEFORE_THRES_INT_MODE;
				}
				else
				{
					//Configure AD7147 in threshold interrupt driven mode
					if(InterruptCounterForThresIntMode_1>0)
						InterruptCounterForThresIntMode_1--;
					if((AD7147Registers_1[STAGE_HIGH_LIMIT_INT]==0x0000) && InterruptCounterForThresIntMode_1==0)
					{
						AD7147Registers_1[STAGE_LOW_INT_EN] |= POWER_UP_INTERRUPT;
						AD7147Registers_1[STAGE_HIGH_INT_EN] |= STAGE_HIGH_INT_EN_VALUE;
						AD7147Registers_1[STAGE_COMPLETE_INT_EN]=0x0000;
						WriteToAD7147(STAGE_LOW_INT_EN, 3, AD7147Registers_1, STAGE_LOW_INT_EN, AD7147_1_ADDRESS);
						slider_switching_approval = THRESHOLD;
						/*if(connected_slider == SLIDER_TWO)
						{
							connected_slider = SLIDER_ONE;
						}	
						else
						{
							connected_slider = SLIDER_TWO;
						}*/
					}
				}
			}				
		}
		else if(dev_address == AD7147_2_ADDRESS)
		{
			//Read thresholds registers
			ReadFromAD7147(STAGE_LOW_LIMIT_INT, 3, AD7147Registers_2, STAGE_LOW_LIMIT_INT, AD7147_2_ADDRESS);	
			if(InterruptCounterForInitialisation_2 < (NB_OF_INT+2))
			{		
				if (InterruptCounterForInitialisation_2==(NB_OF_INT-1))
				{
					//================================
					//= Put initialisation code here =
					//================================
					InitialiseSlider(AD7147_2_ADDRESS);
				}
				InterruptCounterForInitialisation_2++;
			}		
			if(InterruptCounterForInitialisation_2>=NB_OF_INT)
			{
				//============================
				//= Recalibrate if required. =
				//============================
				
				if ((AD7147Registers_2[STAGE_LOW_LIMIT_INT] & POWER_UP_INTERRUPT) != 0x0000)
				{
					ForceCalibration(AD7147_2_ADDRESS);
					InterruptCounterForInitialisation_2 = 0;
				}
				else
				{
					//Get a new slider position status
					g_SliderStatus_2 = GetNewSliderUpdate(AD7147_2_ADDRESS, AD7147Registers_2); //"g_SliderStatus" is updated from this line on.
				}
				//=======================
				// Change interrupt mode
				//=======================
				//Configure AD7147 in EOC interrupt driven mode
				if(((AD7147Registers_2[STAGE_HIGH_LIMIT_INT] & POWER_UP_INTERRUPT)!=0x0000) || 
					((AD7147Registers_2[STAGE_LOW_LIMIT_INT] & POWER_UP_INTERRUPT)!=0x0000))
				{
					if(AD7147Registers_2[STAGE_COMPLETE_INT_EN]==0x0000)
					{
						AD7147Registers_2[STAGE_LOW_INT_EN] &= 0xF000;
						AD7147Registers_2[STAGE_HIGH_INT_EN] &= 0xF000;
						AD7147Registers_2[STAGE_COMPLETE_INT_EN]=0x0001;
						WriteToAD7147(STAGE_LOW_INT_EN, 3, AD7147Registers_2, STAGE_LOW_INT_EN, AD7147_2_ADDRESS);
						slider_switching_approval = COMPLETE;
					}
					InterruptCounterForThresIntMode_2 = NUMBER_OF_INTS_BEFORE_THRES_INT_MODE;
				}
				else
				{
					//Configure AD7147 in threshold interrupt driven mode
					if(InterruptCounterForThresIntMode_2>0)
						InterruptCounterForThresIntMode_2--;
					if((AD7147Registers_2[STAGE_HIGH_LIMIT_INT]==0x0000) && InterruptCounterForThresIntMode_2==0)
					{
						AD7147Registers_2[STAGE_LOW_INT_EN] |= POWER_UP_INTERRUPT;
						AD7147Registers_2[STAGE_HIGH_INT_EN] |= STAGE_HIGH_INT_EN_VALUE;
						AD7147Registers_2[STAGE_COMPLETE_INT_EN]=0x0000;
						WriteToAD7147(STAGE_LOW_INT_EN, 3, AD7147Registers_2, STAGE_LOW_INT_EN, AD7147_2_ADDRESS);
						slider_switching_approval = THRESHOLD;
						/*if(connected_slider == SLIDER_TWO)
						{
							connected_slider = SLIDER_ONE;
						}	
						else
						{
							connected_slider = SLIDER_TWO;
						}*/
					}
				}
			}//End if(InterruptCounterForInitialisation>=NB_OF_INT)				
		}	
	}





	//ForceCalibration();
	//-----------------------------------------------------------------
	// Function called by ServiceAD7147Isr when there is a touch on 
	// power up or when there is a problem with the calibration.
	//-----------------------------------------------------------------
	void ForceCalibration(uint8_t dev_address)
	{
		if(dev_address == AD7147_1_ADDRESS)
		{
				ReadFromAD7147(AMB_COMP_CTRL0, 1, AD7147Registers_1, AMB_COMP_CTRL0, dev_address);
				AD7147Registers_1[AMB_COMP_CTRL0] |= 0x4000;//Set "forced cal" bit
				WriteToAD7147(AMB_COMP_CTRL0, 1, AD7147Registers_1, AMB_COMP_CTRL0, dev_address);
		}
		else if(dev_address == AD7147_2_ADDRESS)
		{
				ReadFromAD7147(AMB_COMP_CTRL0, 1, AD7147Registers_2, AMB_COMP_CTRL0, dev_address);
				AD7147Registers_2[AMB_COMP_CTRL0] |= 0x4000;//Set "forced cal" bit
				WriteToAD7147(AMB_COMP_CTRL0, 1, AD7147Registers_2, AMB_COMP_CTRL0, dev_address);
		}	
	}

	//---------------------
	//Function definitions
	//---------------------
	WORD GetNewSliderUpdate(uint8_t dev_address, WORD *reg_container)
	{
		BYTE i;
		WORD  AmbientValueAddress;
		BYTE  TouchError;	
		WORD  TempAveragePosition;
		DWORD  TempSliderPosition;
		DWORD  A_parameter;
		DWORD  B_parameter;
		
		
		/*for(uint16_t t = 0; t < 6; t++)
		{
			HAL_UART_Transmit(&huart5, AD7147_read_from_reg(ADCRESULT_S0 + t), 2, 1);
		}*/

		
		if(dev_address == AD7147_1_ADDRESS)
		{
			//FIRST	
			if((reg_container[STAGE_HIGH_LIMIT_INT] & STAGES_USED_FOR_SLIDER) != 0x0000)
			{	
			
				//If any slider stages is activated then we read data from the AD7147
				ReadFromAD7147(ADCRESULT_S0, NB_OF_SENSORS_FOR_SLIDER, reg_container, ADCRESULT_S0, dev_address);		
				
				AmbientValueAddress=STAGE0_AMBIENT;
				for(i=0;i<NB_OF_SENSORS_FOR_SLIDER;i++)
				{
					ReadFromAD7147(AmbientValueAddress, 1, AmbientValues_1, i, dev_address);
					AmbientValueAddress=AmbientValueAddress+NB_OF_REGS_PER_STAGE;
				}
			}			
			//Calculate the sensor responses
			for(i=0;i<NB_OF_SENSORS_FOR_SLIDER;i++)
			{			
				if(reg_container[ADCRESULT_S0+i]>AmbientValues_1[i]) 
					SensorValues_1[i]=abs(reg_container[ADCRESULT_S0+i]-AmbientValues_1[i]);
				else
					SensorValues_1[i]=0;
			}	
			//Check for touch errors	
			TouchError = FindHighestAndLowestStagesUsed(reg_container[STAGE_HIGH_LIMIT_INT], LOWER_SENSOR_STAGE, 
																NB_OF_SENSORS_FOR_SLIDER, NUMBER_OF_SENSORS_ON_FOR_BIG_FINGER);
			
			//= Slider activation
			//===================
			if(((reg_container[STAGE_HIGH_LIMIT_INT] & STAGES_USED_FOR_SLIDER) != 0x0000))
			{
				TappingFlag_1=0;//Touching therefore clear the tapping flag.
				RecordSliderCounterOnce_1 = 1;
				SliderStatus_1 &= 0xBFFF;
				//On Touch
				FastScrollDetected_1=0;				
				
				//Check if the user left enough time between 2 taps.
				if (NoTouchCounterOnSlider_1 > T_MIN_NO_TOUCHING)
					ReturnTappingCounterOnSlider_1=DISPLAY_AFTER_A_TAP; //Reset counter for displaying the tap
				else
					ReturnTappingCounterOnSlider_1=0;
				
				if (SliderTouchDownCounter_1<100)
				{			
					//Touching for more than 0.5sec, hence clear the NoTouchCounter
					if (SliderTouchDownCounter_1 > (T_MIN_TOUCHING+T_MAX_TOUCHING))
						NoTouchCounterOnSlider_1=0;
					
					SliderTouchDownCounter_1++;
				}
				else
				{
					NoTouchCounterOnSlider_1=0;
					SliderTouchDownCounter_1=MAX_TOUCHDOWN_COUNTER_TIME;
				}
				
				if (SliderTouchDownCounter_1>(T_MIN_TOUCHING))
					SliderFlag_1=1;
				
			}
			else if((AD7147Registers_2[STAGE_HIGH_LIMIT_INT] & STAGES_USED_FOR_SLIDER) == 0x0000)
			{	
				//Listbox Variables
				ListBoxUpdateCounterValue_1=LISTBOX_SLOW_UPDATE;
				ListBoxUpdateCounter_1=0;
				PositionOnFirstTouchRecorded_1=0;
				
				//Slider Activation Variables
				SliderFlag_1=0;
				FirstTimeSliderTouched_1=0;		
				SliderStatus_1 &= 0x7FFF;
				
				//Keep scrolling through list box if we were moving fast on the slider
				if (RecordSliderCounterOnce_1==1)
				{
					RecordSliderCounterOnce_1=0;
					//Fast scroll detection on Y axis
					//===============================
					if (SliderTouchDownCounter_1 > T_MIN_TOUCHING && SliderTouchDownCounter_1 < T_MAX_TOUCHING)
					{
						NumberOfUpdates_1 = abs(PositionOnActivation_1 - SliderPosition_1) / DisplayItemsResolution_1;

						if (abs(PositionOnActivation_1 - SliderPosition_1) > (NUMBER_OF_POSITIONS>>4))
							NumberOfUpdates_1 = NumberOfUpdates_1 + 5;

						if (NumberOfUpdates_1>1 && NumberOfUpdates_1<9)
							NumberOfUpdates_1 = MIN_NUMBER_OF_UPDATES;
						else if (NumberOfUpdates_1==1)
							NumberOfUpdates_1 = 0;
						
						if (SliderPosition_1 > PositionOnActivation_1)
							FastScrollDirection_1 = DOWN;
						else if (SliderPosition_1 < PositionOnActivation_1)
							FastScrollDirection_1 = UP;
						FastScrollUpdateCounter_1=LISTBOX_QUICK_UPDATE;
						
						//Check if there will be enough interrupts after lifting off before to switch to threshold mode.
						MinimalNumberOfInterruptsAfterLiftingOff_1 = NumberOfUpdates_1 * LISTBOX_QUICK_UPDATE;
						if ((MinimalNumberOfInterruptsAfterLiftingOff_1) > NUMBER_OF_INTS_BEFORE_THRES_INT_MODE)
							MinimalNumberOfInterruptsAfterLiftingOff_1 = MinimalNumberOfInterruptsAfterLiftingOff_1 + 10;
										
						FastScrollDetected_1=1;
						
						if (MovedSinceActivation_1==0)
						{
							NumberOfUpdates_1=0;
							FastScrollDetected_1=0;
						}
					}
					else
					{
						FastScrollDetected_1=0;
						NumberOfUpdates_1=0;
					}
				}
				
				MovedSinceActivation_1=0;

				//= Work out the tap
				//==================
				if (NoTouchCounterOnSlider_1<MAX_NO_TOUCH_COUNTER)
					NoTouchCounterOnSlider_1++;
				
				if (SliderTouchDownCounter_1>T_MIN_TOUCHING && SliderTouchDownCounter_1<T_MAX_TOUCHING &&
					NoTouchCounterOnSlider_1 > SliderTouchDownCounter_1)
				{				
					//Set a uint8_t for a certain number of interrupts so that the user applications sees it.
					//This is done with a decounter that is reloaded when we are touching
					if (ReturnTappingCounterOnSlider_1>0 && EnableTapDisplayOnSlider_1==1)
					{
						ReturnTappingCounterOnSlider_1--;
						TappingFlag_1=1;	//Tap detected here
					}
					else
					{
						SliderTouchDownCounter_1=0;
						TappingFlag_1=0;
					}
				}
				else
				{
					if (SliderTouchDownCounter_1>T_MAX_TOUCHING)
						SliderTouchDownCounter_1=0;
					
					ReturnTappingCounterOnSlider_1=0;
					TappingFlag_1=0;//Cleared tapping uint8_t after lift off 
				}
			}	
				
			if(SliderFlag_1==1)
			{
				//===========================================================
				//============== ABSOLUTE POSITION CALCULATION ==============
				//===========================================================
				if((TouchError & 0x01)!=0x01)
				{
					//===========================================================
					//Find peak among the response of the measurements on Slider
					//===========================================================
					SensorWithHighestValue_1=0;
					for (i=0;i<NB_OF_SENSORS_FOR_SLIDER;i++)
					{
						if (i<(NB_OF_SENSORS_FOR_SLIDER-1))
						{
							if ((SensorValues_1[i] > SensorValues_1[i+1]) && 
								(SensorValues_1[i] > SensorValues_1[SensorWithHighestValue_1]))
								SensorWithHighestValue_1=i;
						}
						else if (i==(NB_OF_SENSORS_FOR_SLIDER-1))
						{
							if ((SensorValues_1[i] > SensorValues_1[i-1]) && 
								(SensorValues_1[i] > SensorValues_1[SensorWithHighestValue_1]))
								SensorWithHighestValue_1=i;
						}
					}

					//==================================================================
					//============== SLIDER ABSOLUTE POSITION CALCULATION ==============
					//==================================================================
					switch (SensorWithHighestValue_1)
					{
						case 0:
							//Calculate "A" parameter
							A_parameter = SensorValues_1[1];
							//Calculate "B" parameter
							B_parameter = SensorValues_1[0] + SensorValues_1[1];					
												
							TempSliderPosition = (PIXEL_RESOLUTION * A_parameter);
							TempSliderPosition = TempSliderPosition / B_parameter;
							
							if(TempSliderPosition>=DISPLACEMENT_OFFSET)
								SliderPosition_1=(WORD)(TempSliderPosition-DISPLACEMENT_OFFSET);
							else
								SliderPosition_1=(WORD)TempSliderPosition;						
							break;

						case (NB_OF_SENSORS_FOR_SLIDER-1):
							//Calculate "A" parameter
							A_parameter = SensorValues_1[NB_OF_SENSORS_FOR_SLIDER-1] * (NB_OF_SENSORS_FOR_SLIDER-1);
							A_parameter = A_parameter + SensorValues_1[NB_OF_SENSORS_FOR_SLIDER-2] * (NB_OF_SENSORS_FOR_SLIDER-2);
							//Calculate "B" parameter
							B_parameter = SensorValues_1[NB_OF_SENSORS_FOR_SLIDER-1] + SensorValues_1[NB_OF_SENSORS_FOR_SLIDER-2];// + SensorValues_1[NB_OF_SENSORS_FOR_SLIDER-3];
												
							TempSliderPosition = (PIXEL_RESOLUTION * A_parameter);
							TempSliderPosition = TempSliderPosition / B_parameter;					
							
							if(TempSliderPosition>=DISPLACEMENT_OFFSET)
								SliderPosition_1=(WORD)(TempSliderPosition-DISPLACEMENT_OFFSET);
							else
								SliderPosition_1=(WORD)TempSliderPosition;
							break;
						default:
							//Calculate "A" parameter
							A_parameter = SensorValues_1[SensorWithHighestValue_1-1] * (SensorWithHighestValue_1-1);
							A_parameter = A_parameter + SensorValues_1[SensorWithHighestValue_1] * (SensorWithHighestValue_1);
							A_parameter = A_parameter + SensorValues_1[SensorWithHighestValue_1+1] * (SensorWithHighestValue_1+1);
							
							//Calculate "B" parameter
							B_parameter = SensorValues_1[SensorWithHighestValue_1-1] + SensorValues_1[SensorWithHighestValue_1] + SensorValues_1[SensorWithHighestValue_1+1];
							
							TempSliderPosition = (PIXEL_RESOLUTION * A_parameter);
							TempSliderPosition = TempSliderPosition / B_parameter;
							
							if(TempSliderPosition>=DISPLACEMENT_OFFSET)
								SliderPosition_1=(WORD)(TempSliderPosition-DISPLACEMENT_OFFSET);
							else
								SliderPosition_1=(WORD)TempSliderPosition;
							break;
					}
					
					if(FirstTimeSliderTouched_1==0)
					{
						FirstTimeSliderTouched_1=1;
						if ((SliderPosition_1 > (NUMBER_OF_POSITIONS-5)) || (SliderPosition_1 < 5))
							ListBoxUpdateCounterValue_1=LISTBOX_SLOW_UPDATE;
						else
							ListBoxUpdateCounterValue_1 = LISTBOX_QUICK_UPDATE;
					}
					
					//= Initialise some of the position variables on activation
					//=========================================================
					if (PositionOnFirstTouchRecorded_1==0)
					{
						PositionOnFirstTouchRecorded_1=1;
						PositionOnFirstTouch_1 = SliderPosition_1;
						PositionOnActivation_1 = SliderPosition_1;
						AveragePosition_1 = SliderPosition_1;
					}
					else if (PositionOnFirstTouchRecorded_1==1)
					{
						//= Filter out slider response
						//============================
						TempAveragePosition = AveragePosition_1 * 6 + SliderPosition_1 * 4.0;
						AveragePosition_1 = TempAveragePosition / 10;
						
						if (AveragePosition_1 > NUMBER_OF_POSITIONS)
							AveragePosition_1 = NUMBER_OF_POSITIONS;
						
						//= Code for relative positions for the slider
						//============================================
						//Move through items in the listbox as long as no buttons are being set
						if (SliderPosition_1 < 5)
						{	//Auto scroll up
							ListBoxUpdateCounter_1++;
							if (ListBoxUpdateCounter_1 == ListBoxUpdateCounterValue_1)
							{
								ListBoxUpdateCounter_1=0;
								ListBoxUpdateCounterValue_1=LISTBOX_QUICK_UPDATE;
								//Go up
								SliderStatus_1 &= 0xCFFF;
								SliderStatus_1 |= 0x2000;
							}
							else
							{
								SliderStatus_1 &= 0xCFFF;
							}
						}				
						else if(SliderPosition_1 > (NUMBER_OF_POSITIONS-5) )
						{	//Auto scroll down
							ListBoxUpdateCounter_1++;
							if (ListBoxUpdateCounter_1 == ListBoxUpdateCounterValue_1)
							{
								ListBoxUpdateCounter_1=0;
								ListBoxUpdateCounterValue_1=LISTBOX_QUICK_UPDATE;
								//Go down
								SliderStatus_1 &= 0xCFFF;
								SliderStatus_1 |= 0x1000;
							}
							else
							{
								SliderStatus_1 &= 0xCFFF;
							}
						}
						else
						{
							if (abs(SliderPosition_1 - PositionOnFirstTouch_1) > DisplayItemsResolution_1)
							{
								//Create list box commands for the Y axis
								if (SliderPosition_1 > PositionOnFirstTouch_1)
								{	//Go down
									SliderStatus_1 &= 0xCFFF;
									SliderStatus_1 |= 0x1000;
								}
								else if (SliderPosition_1 < PositionOnFirstTouch_1)
								{	//Go up
									SliderStatus_1 &= 0xCFFF;
									SliderStatus_1 |= 0x2000;
								}
								PositionOnFirstTouch_1 = SliderPosition_1;
								MovedSinceActivation_1=1;
							}
							else
							{
								SliderStatus_1 &= 0xCFFF;
							}
						}
					}//End else if (PositionOnFirstTouchRecorded_1==1)
				}//End if ((TouchError & 0x01)!=0x01)
			}

			//= Clear tap if we're scrolling
			//==============================
			//We have a new position, if it is different from the previous one and we are still within the
			//the tapping time, then we check the positions. If we are moving, then we clear the tapping uint8_t.
			if( ((TouchError& 0x1)!=0x1) && SliderTouchDownCounter_1<T_MAX_TOUCHING)
			{
				if(MovedSinceActivation_1==1)
				{
					TappingFlag_1=0; //Clear tapping uint8_t on slider
					EnableTapDisplayOnSlider_1=0;	//We're moving, so don't display the tap;
				}
				else
					EnableTapDisplayOnSlider_1=1;	//We haven't moved by far enough, so a tap is still valid
			}
			
			//= Format the position data
			//==========================
			SliderStatus_1 &= 0xF3FF;
			SliderStatus_1 |= ((TouchError <<2)<<8);
			
			if (SliderFlag_1==1)
			{
				SliderStatus_1 |= 0x8000;
				//Compute position registers
				if(((TouchError & 0x01)!=0x01))
				{
					SliderStatus_1 &= 0xBC00;
					SliderStatus_1 |= AveragePosition_1;
				}
			}
			else	//Not touching...
			{
				//Clear finger valid uint8_t and go up go down commands
				SliderStatus_1 &= 0x0FFF;
				//Return Tap
				if(TappingFlag_1==1)
					SliderStatus_1 |= 0x4000;

				if (FastScrollDetected_1==1)
				{
					if (NumberOfUpdates_1>0)
					{
						if (FastScrollUpdateCounter_1>0)
						{
							SliderStatus_1 &= 0xCFFF;
							FastScrollUpdateCounter_1--;
						}
						else
						{
							NumberOfUpdates_1--;
							FastScrollUpdateCounter_1=3;
							if (FastScrollDirection_1==UP)
							{
								//Go up
								SliderStatus_1 &= 0xCFFF;
								SliderStatus_1 |= 0x2000;
							}
							else if (FastScrollDirection_1==DOWN)
							{
								//Go down
								SliderStatus_1 &= 0xCFFF;
								SliderStatus_1 |= 0x1000;
							}
						}
					}
					else
					{
						FastScrollDetected_1=0;
						SliderStatus_1 &= 0xCFFF;
					}
				}
			}
			return(SliderStatus_1);
		}
		else if(dev_address == AD7147_2_ADDRESS)
		{
			//SECOND		
			if((reg_container[STAGE_HIGH_LIMIT_INT] & STAGES_USED_FOR_SLIDER) != 0x0000)
			{	
			
				//If any slider stages is activated then we read data from the AD7147
				ReadFromAD7147(ADCRESULT_S0, NB_OF_SENSORS_FOR_SLIDER, reg_container, ADCRESULT_S0, dev_address);		
				
				AmbientValueAddress=STAGE0_AMBIENT;
				for(i=0;i<NB_OF_SENSORS_FOR_SLIDER;i++)
				{
					ReadFromAD7147(AmbientValueAddress, 1, AmbientValues_2, i, dev_address);
					AmbientValueAddress=AmbientValueAddress+NB_OF_REGS_PER_STAGE;
				}
			}			
			//Calculate the sensor responses
			for(i=0;i<NB_OF_SENSORS_FOR_SLIDER;i++)
			{			
				if(reg_container[ADCRESULT_S0+i]>AmbientValues_2[i]) 
					SensorValues_2[i]=abs(reg_container[ADCRESULT_S0+i]-AmbientValues_2[i]);
				else
					SensorValues_2[i]=0;
			}	
			//Check for touch errors	
			TouchError = FindHighestAndLowestStagesUsed(reg_container[STAGE_HIGH_LIMIT_INT], LOWER_SENSOR_STAGE, 
																NB_OF_SENSORS_FOR_SLIDER, NUMBER_OF_SENSORS_ON_FOR_BIG_FINGER);
			
			//= Slider activation
			//===================
			if(((reg_container[STAGE_HIGH_LIMIT_INT] & STAGES_USED_FOR_SLIDER) != 0x0000))
			{
				TappingFlag_2=0;//Touching therefore clear the tapping flag.
				RecordSliderCounterOnce_2 = 1;
				SliderStatus_2 &= 0xBFFF;
				//On Touch
				FastScrollDetected_2=0;				
				
				//Check if the user left enough time between 2 taps.
				if (NoTouchCounterOnSlider_2 > T_MIN_NO_TOUCHING)
					ReturnTappingCounterOnSlider_2=DISPLAY_AFTER_A_TAP; //Reset counter for displaying the tap
				else
					ReturnTappingCounterOnSlider_2=0;
				
				if (SliderTouchDownCounter_2<100)
				{			
					//Touching for more than 0.5sec, hence clear the NoTouchCounter
					if (SliderTouchDownCounter_2 > (T_MIN_TOUCHING+T_MAX_TOUCHING))
						NoTouchCounterOnSlider_2=0;
					
					SliderTouchDownCounter_2++;
				}
				else
				{
					NoTouchCounterOnSlider_2=0;
					SliderTouchDownCounter_2=MAX_TOUCHDOWN_COUNTER_TIME;
				}
				
				if (SliderTouchDownCounter_2>(T_MIN_TOUCHING))
					SliderFlag_2=1;
				
			}
			else if((AD7147Registers_2[STAGE_HIGH_LIMIT_INT] & STAGES_USED_FOR_SLIDER) == 0x0000)
			{	
				//Listbox Variables
				ListBoxUpdateCounterValue_2=LISTBOX_SLOW_UPDATE;
				ListBoxUpdateCounter_2=0;
				PositionOnFirstTouchRecorded_2=0;
				
				//Slider Activation Variables
				SliderFlag_2=0;
				FirstTimeSliderTouched_2=0;		
				SliderStatus_2 &= 0x7FFF;
				
				//Keep scrolling through list box if we were moving fast on the slider
				if (RecordSliderCounterOnce_2==1)
				{
					RecordSliderCounterOnce_2=0;
					//Fast scroll detection on Y axis
					//===============================
					if (SliderTouchDownCounter_2 > T_MIN_TOUCHING && SliderTouchDownCounter_2 < T_MAX_TOUCHING)
					{
						NumberOfUpdates_2 = abs(PositionOnActivation_2 - SliderPosition_2) / DisplayItemsResolution_2;

						if (abs(PositionOnActivation_2 - SliderPosition_2) > (NUMBER_OF_POSITIONS>>4))
							NumberOfUpdates_2 = NumberOfUpdates_2 + 5;

						if (NumberOfUpdates_2>1 && NumberOfUpdates_2<9)
							NumberOfUpdates_2 = MIN_NUMBER_OF_UPDATES;
						else if (NumberOfUpdates_2==1)
							NumberOfUpdates_2 = 0;
						
						if (SliderPosition_2 > PositionOnActivation_2)
							FastScrollDirection_2 = DOWN;
						else if (SliderPosition_2 < PositionOnActivation_2)
							FastScrollDirection_2 = UP;
						FastScrollUpdateCounter_2=LISTBOX_QUICK_UPDATE;
						
						//Check if there will be enough interrupts after lifting off before to switch to threshold mode.
						MinimalNumberOfInterruptsAfterLiftingOff_2 = NumberOfUpdates_2 * LISTBOX_QUICK_UPDATE;
						if ((MinimalNumberOfInterruptsAfterLiftingOff_2) > NUMBER_OF_INTS_BEFORE_THRES_INT_MODE)
							MinimalNumberOfInterruptsAfterLiftingOff_2 = MinimalNumberOfInterruptsAfterLiftingOff_2 + 10;
										
						FastScrollDetected_2=1;
						
						if (MovedSinceActivation_2==0)
						{
							NumberOfUpdates_2=0;
							FastScrollDetected_2=0;
						}
					}
					else
					{
						FastScrollDetected_2=0;
						NumberOfUpdates_2=0;
					}
				}
				
				MovedSinceActivation_2=0;

				//= Work out the tap
				//==================
				if (NoTouchCounterOnSlider_2<MAX_NO_TOUCH_COUNTER)
					NoTouchCounterOnSlider_2++;
				
				if (SliderTouchDownCounter_2>T_MIN_TOUCHING && SliderTouchDownCounter_2<T_MAX_TOUCHING &&
					NoTouchCounterOnSlider_2 > SliderTouchDownCounter_2)
				{				
					//Set a uint8_t for a certain number of interrupts so that the user applications sees it.
					//This is done with a decounter that is reloaded when we are touching
					if (ReturnTappingCounterOnSlider_2>0 && EnableTapDisplayOnSlider_2==1)
					{
						ReturnTappingCounterOnSlider_2--;
						TappingFlag_2=1;	//Tap detected here
					}
					else
					{
						SliderTouchDownCounter_2=0;
						TappingFlag_2=0;
					}
				}
				else
				{
					if (SliderTouchDownCounter_2>T_MAX_TOUCHING)
						SliderTouchDownCounter_2=0;
					
					ReturnTappingCounterOnSlider_2=0;
					TappingFlag_2=0;//Cleared tapping uint8_t after lift off 
				}
			}	
				
			if(SliderFlag_2==1)
			{
				//===========================================================
				//============== ABSOLUTE POSITION CALCULATION ==============
				//===========================================================
				if((TouchError & 0x01)!=0x01)
				{
					//===========================================================
					//Find peak among the response of the measurements on Slider
					//===========================================================
					SensorWithHighestValue_2=0;
					for (i=0;i<NB_OF_SENSORS_FOR_SLIDER;i++)
					{
						if (i<(NB_OF_SENSORS_FOR_SLIDER-1))
						{
							if ((SensorValues_2[i] > SensorValues_2[i+1]) && 
								(SensorValues_2[i] > SensorValues_2[SensorWithHighestValue_2]))
								SensorWithHighestValue_2=i;
						}
						else if (i==(NB_OF_SENSORS_FOR_SLIDER-1))
						{
							if ((SensorValues_2[i] > SensorValues_2[i-1]) && 
								(SensorValues_2[i] > SensorValues_2[SensorWithHighestValue_2]))
								SensorWithHighestValue_2=i;
						}
					}

					//==================================================================
					//============== SLIDER ABSOLUTE POSITION CALCULATION ==============
					//==================================================================
					switch (SensorWithHighestValue_2)
					{
						case 0:
							//Calculate "A" parameter
							A_parameter = SensorValues_2[1];
							//Calculate "B" parameter
							B_parameter = SensorValues_2[0] + SensorValues_2[1];					
												
							TempSliderPosition = (PIXEL_RESOLUTION * A_parameter);
							TempSliderPosition = TempSliderPosition / B_parameter;
							
							if(TempSliderPosition>=DISPLACEMENT_OFFSET)
								SliderPosition_2=(WORD)(TempSliderPosition-DISPLACEMENT_OFFSET);
							else
								SliderPosition_2=(WORD)TempSliderPosition;						
							break;

						case (NB_OF_SENSORS_FOR_SLIDER-1):
							//Calculate "A" parameter
							A_parameter = SensorValues_2[NB_OF_SENSORS_FOR_SLIDER-1] * (NB_OF_SENSORS_FOR_SLIDER-1);
							A_parameter = A_parameter + SensorValues_2[NB_OF_SENSORS_FOR_SLIDER-2] * (NB_OF_SENSORS_FOR_SLIDER-2);
							//Calculate "B" parameter
							B_parameter = SensorValues_2[NB_OF_SENSORS_FOR_SLIDER-1] + SensorValues_2[NB_OF_SENSORS_FOR_SLIDER-2];// + SensorValues_2[NB_OF_SENSORS_FOR_SLIDER-3];
												
							TempSliderPosition = (PIXEL_RESOLUTION * A_parameter);
							TempSliderPosition = TempSliderPosition / B_parameter;					
							
							if(TempSliderPosition>=DISPLACEMENT_OFFSET)
								SliderPosition_2=(WORD)(TempSliderPosition-DISPLACEMENT_OFFSET);
							else
								SliderPosition_2=(WORD)TempSliderPosition;
							break;
						default:
							//Calculate "A" parameter
							A_parameter = SensorValues_2[SensorWithHighestValue_2-1] * (SensorWithHighestValue_2-1);
							A_parameter = A_parameter + SensorValues_2[SensorWithHighestValue_2] * (SensorWithHighestValue_2);
							A_parameter = A_parameter + SensorValues_2[SensorWithHighestValue_2+1] * (SensorWithHighestValue_2+1);
							
							//Calculate "B" parameter
							B_parameter = SensorValues_2[SensorWithHighestValue_2-1] + SensorValues_2[SensorWithHighestValue_2] + SensorValues_2[SensorWithHighestValue_2+1];
							
							TempSliderPosition = (PIXEL_RESOLUTION * A_parameter);
							TempSliderPosition = TempSliderPosition / B_parameter;
							
							if(TempSliderPosition>=DISPLACEMENT_OFFSET)
								SliderPosition_2=(WORD)(TempSliderPosition-DISPLACEMENT_OFFSET);
							else
								SliderPosition_2=(WORD)TempSliderPosition;
							break;
					}
					
					if(FirstTimeSliderTouched_2==0)
					{
						FirstTimeSliderTouched_2=1;
						if ((SliderPosition_2 > (NUMBER_OF_POSITIONS-5)) || (SliderPosition_2 < 5))
							ListBoxUpdateCounterValue_2=LISTBOX_SLOW_UPDATE;
						else
							ListBoxUpdateCounterValue_2 = LISTBOX_QUICK_UPDATE;
					}
					
					//= Initialise some of the position variables on activation
					//=========================================================
					if (PositionOnFirstTouchRecorded_2==0)
					{
						PositionOnFirstTouchRecorded_2=1;
						PositionOnFirstTouch_2 = SliderPosition_2;
						PositionOnActivation_2 = SliderPosition_2;
						AveragePosition_2 = SliderPosition_2;
					}
					else if (PositionOnFirstTouchRecorded_2==1)
					{
						//= Filter out slider response
						//============================
						TempAveragePosition = AveragePosition_2 * 6 + SliderPosition_2 * 4.0;
						AveragePosition_2 = TempAveragePosition / 10;
						
						if (AveragePosition_2 > NUMBER_OF_POSITIONS)
							AveragePosition_2 = NUMBER_OF_POSITIONS;
						
						//= Code for relative positions for the slider
						//============================================
						//Move through items in the listbox as long as no buttons are being set
						if (SliderPosition_2 < 5)
						{	//Auto scroll up
							ListBoxUpdateCounter_2++;
							if (ListBoxUpdateCounter_2 == ListBoxUpdateCounterValue_2)
							{
								ListBoxUpdateCounter_2=0;
								ListBoxUpdateCounterValue_2=LISTBOX_QUICK_UPDATE;
								//Go up
								SliderStatus_2 &= 0xCFFF;
								SliderStatus_2 |= 0x2000;
							}
							else
							{
								SliderStatus_2 &= 0xCFFF;
							}
						}				
						else if(SliderPosition_2 > (NUMBER_OF_POSITIONS-5) )
						{	//Auto scroll down
							ListBoxUpdateCounter_2++;
							if (ListBoxUpdateCounter_2 == ListBoxUpdateCounterValue_2)
							{
								ListBoxUpdateCounter_2=0;
								ListBoxUpdateCounterValue_2=LISTBOX_QUICK_UPDATE;
								//Go down
								SliderStatus_2 &= 0xCFFF;
								SliderStatus_2 |= 0x1000;
							}
							else
							{
								SliderStatus_2 &= 0xCFFF;
							}
						}
						else
						{
							if (abs(SliderPosition_2 - PositionOnFirstTouch_2) > DisplayItemsResolution_2)
							{
								//Create list box commands for the Y axis
								if (SliderPosition_2 > PositionOnFirstTouch_2)
								{	//Go down
									SliderStatus_2 &= 0xCFFF;
									SliderStatus_2 |= 0x1000;
								}
								else if (SliderPosition_2 < PositionOnFirstTouch_2)
								{	//Go up
									SliderStatus_2 &= 0xCFFF;
									SliderStatus_2 |= 0x2000;
								}
								PositionOnFirstTouch_2 = SliderPosition_2;
								MovedSinceActivation_2=1;
							}
							else
							{
								SliderStatus_2 &= 0xCFFF;
							}
						}
					}//End else if (PositionOnFirstTouchRecorded_2==1)
				}//End if ((TouchError & 0x01)!=0x01)
			}

			//= Clear tap if we're scrolling
			//==============================
			//We have a new position, if it is different from the previous one and we are still within the
			//the tapping time, then we check the positions. If we are moving, then we clear the tapping uint8_t.
			if( ((TouchError& 0x1)!=0x1) && SliderTouchDownCounter_2<T_MAX_TOUCHING)
			{
				if(MovedSinceActivation_2==1)
				{
					TappingFlag_2=0; //Clear tapping uint8_t on slider
					EnableTapDisplayOnSlider_2=0;	//We're moving, so don't display the tap;
				}
				else
					EnableTapDisplayOnSlider_2=1;	//We haven't moved by far enough, so a tap is still valid
			}
			
			//= Format the position data
			//==========================
			SliderStatus_2 &= 0xF3FF;
			SliderStatus_2 |= ((TouchError <<2)<<8);
			
			if (SliderFlag_2==1)
			{
				SliderStatus_2 |= 0x8000;
				//Compute position registers
				if(((TouchError & 0x01)!=0x01))
				{
					SliderStatus_2 &= 0xBC00;
					SliderStatus_2 |= AveragePosition_2;
				}
			}
			else	//Not touching...
			{
				//Clear finger valid uint8_t and go up go down commands
				SliderStatus_2 &= 0x0FFF;
				//Return Tap
				if(TappingFlag_2==1)
					SliderStatus_2 |= 0x4000;

				if (FastScrollDetected_2==1)
				{
					if (NumberOfUpdates_2>0)
					{
						if (FastScrollUpdateCounter_2>0)
						{
							SliderStatus_2 &= 0xCFFF;
							FastScrollUpdateCounter_2--;
						}
						else
						{
							NumberOfUpdates_2--;
							FastScrollUpdateCounter_2=3;
							if (FastScrollDirection_2==UP)
							{
								//Go up
								SliderStatus_2 &= 0xCFFF;
								SliderStatus_2 |= 0x2000;
							}
							else if (FastScrollDirection_2==DOWN)
							{
								//Go down
								SliderStatus_2 &= 0xCFFF;
								SliderStatus_2 |= 0x1000;
							}
						}
					}
					else
					{
						FastScrollDetected_2=0;
						SliderStatus_2 &= 0xCFFF;
					}
				}
			}
			return(SliderStatus_2);
		}	
	return(0);		
	}


	//---------------------------------
	//FindHighestAndLowestStagesUsed()
	//---------------------------------
	//Function that checks if there are 2 fingers touching the slider or if there is one big finger
	BYTE FindHighestAndLowestStagesUsed(WORD InterruptStatusRegister, WORD LowestStageOfTheSensor, BYTE NumberOfStagesUsed, BYTE BigFingerLevel)
	{
		WORD  ShiftValue;
		BYTE  i,  Interruptuint8_tCounter;
		BYTE  LowestSensorTouched=0xFF;
		BYTE  HighestSensorTouched=0xFF;
		uint8_t LowestSensorTouchedFound, HighestSensorTouchedFound;
		uint8_t TwoFingerTouching, BigFingerTouching;
		
		//Initialisation
		LowestSensorTouchedFound=0;
		HighestSensorTouchedFound=0;	
		
		Interruptuint8_tCounter=0;
		TwoFingerTouching=0;
		BigFingerTouching=0;
		
		//For all sensors in use
		//======================
		ShiftValue=LowestStageOfTheSensor;
		for(i=0;i<NumberOfStagesUsed;i++)
		{
			//Found the lowest sensor touched
			if((InterruptStatusRegister & ShiftValue) == ShiftValue) 
			{
				if(LowestSensorTouchedFound==0)
				{
					//Lowest sensor touched is found here
					LowestSensorTouched=i;
					LowestSensorTouchedFound=1;
				}
				//If last sensor is set, then it is also the highest sensor touched
				if(i==(NumberOfStagesUsed-1) && LowestSensorTouchedFound==1 && HighestSensorTouchedFound==0)
				{					
					HighestSensorTouched=i;
					HighestSensorTouchedFound=1;
				}				
			}
			//Found the highest sensor touched
			else if(((InterruptStatusRegister & ShiftValue) == 0) && LowestSensorTouchedFound==1 && HighestSensorTouchedFound==0)
			{
				//Highest sensor touched is found here
				HighestSensorTouched=i-1;
				HighestSensorTouchedFound=1;
			}			
			ShiftValue=ShiftValue<<1;
		}
		
		//Code for detecting if there are 2 fingers on the entire sensor and if there is a big 
		//finger touching it. We count the number of interrupts set and if it is greater than 
		//"BigFingerLevel", then we set the "BigFingerTouching" flag.
		ShiftValue=LowestStageOfTheSensor;		
		for(i=0;i<NumberOfStagesUsed;i++)
		{
			if((InterruptStatusRegister & ShiftValue)==ShiftValue)
			{
				Interruptuint8_tCounter++;
				if((i>HighestSensorTouched) || (i<LowestSensorTouched))
					TwoFingerTouching=1;
			}
			ShiftValue=ShiftValue<<1;
		}

		if(Interruptuint8_tCounter>=BigFingerLevel)
			BigFingerTouching=1;
		//Return error code
		return (((BYTE)TwoFingerTouching) | (((BYTE)BigFingerTouching)<<1));
	}

	//---------------------------------
	//InitialiseSlider()
	//---------------------------------
	//Function that initialises the slider algorithm.
	void InitialiseSlider(uint8_t dev_address)
	{		
		BYTE i;
		WORD  AmbientValueAddress;
			
		AmbientValueAddress = STAGE0_AMBIENT;
		
		if(dev_address == AD7147_1_ADDRESS)
		{
			for (i=0; i<NB_OF_SENSORS_FOR_SLIDER; i++)
			{	
				//Initialise ambient values
				ReadFromAD7147(AmbientValueAddress, 1, AmbientValues_1, i, dev_address);
				AmbientValueAddress=AmbientValueAddress+NB_OF_REGS_PER_STAGE;
				SensorValues_1[i] = 0;
			}

			SliderStatus_1 = 0x0000;
			ListBoxUpdateCounterValue_1 = LISTBOX_SLOW_UPDATE;
			DisplayItemsResolution_1 = NUMBER_OF_POSITIONS / DISPLAY_ITEMS_CONSTANT;	
			EnableTapDisplayOnSlider_1 = 1;
			NumberOfUpdates_1 = 0;
		}
		else if(dev_address == AD7147_2_ADDRESS)
		{
				for (i=0; i<NB_OF_SENSORS_FOR_SLIDER; i++)
			{	
				//Initialise ambient values
				ReadFromAD7147(AmbientValueAddress, 1, AmbientValues_2, i, dev_address);
				AmbientValueAddress=AmbientValueAddress+NB_OF_REGS_PER_STAGE;
				SensorValues_2[i] = 0;
			}

			SliderStatus_2 = 0x0000;
			ListBoxUpdateCounterValue_2 = LISTBOX_SLOW_UPDATE;
			DisplayItemsResolution_2 = NUMBER_OF_POSITIONS / DISPLAY_ITEMS_CONSTANT;	
			EnableTapDisplayOnSlider_2 = 1;
			NumberOfUpdates_2 = 0;
		}	
	}

	void AD7147_OFFSET_CAL(uint8_t dev_address)
	{
		WORD DATA_MASS[NUMBER_OF_AD7147_REGISTERS];
		//WORD AFE_MASS[6];
		WORD AFE_SKELETON = 0x8080;
		WORD AFE_POS = 0;
		WORD AFE_NEG = 0;
		BYTE mode = 0x00;
		std_union AFE_UNION;
		
		uint8_t temp_mass[2];
		std_union temp;
		
		if(dev_address == AD7147_1_ADDRESS)
		{
			for(uint16_t i = 0; i < 12; i++)
			{
				mode = 0x00;
				while(mode != 0x01)
				{	
					HAL_Delay(50);
					ReadFromAD7147(ADCRESULT_S0, 12, DATA_MASS, ADCRESULT_S0, dev_address);			
					for(uint16_t t = 0; t < 12; t++)
					{
						HAL_UART_Transmit(&huart5, AD7147_read_from_reg(ADCRESULT_S0 + t, dev_address), 2, 1);
					}

					AD7147_read_from_reg((STAGE0_CONNECTION + i*8) + 2, dev_address);
					//HAL_UART_Transmit(&huart5, AD7147_read_from_reg(((STAGE0_CONNECTION + 2) + i*8)), 2, 1);
					
					AFE_UNION.cstd[0] = I2C_RX_BUFF[1];
					AFE_UNION.cstd[1] = I2C_RX_BUFF[0];
					AFE_POS = (AFE_UNION.istd & 0x3F00);
					AFE_NEG = (AFE_UNION.istd & 0x003F);
					if((DATA_MASS[ADCRESULT_S0 + i] > OFFSET_HIGH_LIMIT))
					{
						if(AFE_NEG != 0x003F)
						{
							AFE_NEG++;
							AFE_NEG &= 0x003F;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_NEG;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
						else if(AFE_POS != 0x0000)
						{
							AFE_POS -= 0x0100;
							AFE_POS &= 0x3F00;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_POS;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
					}
					else if(DATA_MASS[ADCRESULT_S0 + i] < OFFSET_LOW_LIMIT)
					{	
						if(AFE_NEG != 0x0000)
						{
							AFE_NEG--;
							AFE_NEG &= 0x003F;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_NEG;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
						else if(AFE_POS != 0x3F00)
						{
							AFE_POS += 0x0100;
							AFE_POS &= 0x3F00;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_POS;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
					}
					else 
					{
						mode = 0x01;
						AD7147_calibration_approval_1 = 0x01;
						
					}
				}
			}
		}
		else if(dev_address == AD7147_2_ADDRESS)
		{
			for(uint16_t i = 0; i < 12; i++)
			{
				mode = 0x00;
				while(mode != 0x01)
				{	
					HAL_Delay(50);
					ReadFromAD7147(ADCRESULT_S0, 12, DATA_MASS, ADCRESULT_S0, dev_address);			
					for(uint16_t t = 11; t < 23; t++)
					{
						//HAL_UART_Transmit(&huart5, AD7147_read_from_reg(ADCRESULT_S0 + t, dev_address), 2, 1);
						temp.istd = DATA_MASS[t];
						temp_mass[0] = temp.cstd[1];
						temp_mass[1] = temp.cstd[0];		
						HAL_UART_Transmit(&huart5, temp_mass, 2, 1);

					}

					AD7147_read_from_reg((STAGE0_CONNECTION + i*8) + 2, dev_address);
					HAL_UART_Transmit(&huart5, I2C_RX_BUFF, 2, 1);
					
					AFE_UNION.cstd[0] = I2C_RX_BUFF[1];
					AFE_UNION.cstd[1] = I2C_RX_BUFF[0];
					AFE_POS = (AFE_UNION.istd & 0x3F00);
					AFE_NEG = (AFE_UNION.istd & 0x003F);
					if((DATA_MASS[ADCRESULT_S0 + i] > OFFSET_HIGH_LIMIT))
					{
						if(AFE_NEG != 0x003F)
						{
							AFE_NEG++;
							AFE_NEG &= 0x003F;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_NEG;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
						else if(AFE_POS != 0x0000)
						{
							AFE_POS -= 0x0100;
							AFE_POS &= 0x3F00;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_POS;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
					}
					else if(DATA_MASS[ADCRESULT_S0 + i] < OFFSET_LOW_LIMIT)
					{	
						if(AFE_NEG != 0x0000)
						{
							AFE_NEG--;
							AFE_NEG &= 0x003F;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_NEG;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
						else if(AFE_POS != 0x3F00)
						{
							AFE_POS += 0x0100;
							AFE_POS &= 0x3F00;
							AFE_UNION.istd = 0;
							AFE_UNION.istd |= AFE_SKELETON;
							AFE_UNION.istd |= AFE_POS;
							AD7147_write_to_reg((STAGE0_CONNECTION + i*8 + 2), AFE_UNION.istd, dev_address);	
						}
						/*else
						{
							ForceCalibration(AD7147_2_ADDRESS);						
						}*/
					}
					else 
					{
						mode = 0x01;
						AD7147_calibration_approval_2 = 0x01;
					}
				}
				/*if(connected_slider == SLIDER_TWO)
				{
					slider_2_AFE_CAL[i] = AFE_UNION.istd;
				}
				else if(connected_slider == SLIDER_ONE)
				{
					slider_1_AFE_CAL[i] = AFE_UNION.istd;
				}*/
			}
		}
	}
	
/******************************** END ****************************/
