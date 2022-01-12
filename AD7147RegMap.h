//========================================================================================
//File AD7142RegMap.h
//Data of creation: 12th May 2005
//Author: ADI Limerick
//
//Description 
//===========
//This file contains the AD7142 register and ram maps.
//========================================================================================


// Register map
//=============
#define	PWR_CONTROL					0x0000	// RW	Power & conversion control

#define	STAGE_CAL_EN				0x0001	// RW	Ambient compensation control register 0
#define	AMB_COMP_CTRL0				0x0002	// RW	Ambient compensation control register 1
#define	AMB_COMP_CTRL1				0x0003	// RW	Ambient compensation control register 2
#define	AMB_COMP_CTRL2				0x0004	// RW	Ambient compensation control register 3

#define	STAGE_LOW_INT_EN			0x0005	// RW	Interrupt enable register 0
#define	STAGE_HIGH_INT_EN			0x0006	// RW	Interrupt enable register 1
#define	STAGE_COMPLETE_INT_EN		0x0007	// RW	Interrupt enable register 2
#define	STAGE_LOW_LIMIT_INT			0x0008	// R	Low limit interrupt status register 0
#define	STAGE_HIGH_LIMIT_INT		0x0009	// R	High limit interrupt status register 1
#define	STAGE_COMPLETE_LIMIT_INT	0x000A	// R	Interrupt status register 2

#define	ADCRESULT_S0				0x000B	// R	ADC stage 0 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S1				0x000C	// R	ADC stage 1 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S2				0x000D	// R	ADC stage 2 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S3				0x000E	// R	ADC stage 3 result (uncompensated) actually located in SRAM

#define	ADCRESULT_S4				0x000F	// R	ADC stage 4 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S5				0x0010	// R	ADC stage 5 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S6				0x0011	// R	ADC stage 6 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S7				0x0012	// R	ADC stage 7 result (uncompensated) actually located in SRAM

#define	ADCRESULT_S8				0x0013	// R	ADC stage 8 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S9				0x0014	// R	ADC stage 9 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S10				0x0015	// R	ADC stage 10 result (uncompensated) actually located in SRAM
#define	ADCRESULT_S11				0x0016	// R	ADC stage 11 result (uncompensated) actually located in SRAM

#define	DEVID_REG						0x0017	// R	I.D. Register

#define	THRES_STAT_REG0				0x0040	// R	Current threshold status register 0
#define	THRES_STAT_REG1				0x0041	// R	Current threshold status register 1
#define	PROX_STAT_REG				0x0042	// R	Current proximity status register 2


// Ram map - these registers are defined as we go along
//=====================================================
#define STAGE0_CONNECTION	0x0080
#define STAGE1_CONNECTION	0x0088
#define STAGE2_CONNECTION	0x0090
#define STAGE3_CONNECTION	0x0098
#define STAGE4_CONNECTION	0x00A0
#define STAGE5_CONNECTION	0x00A8
#define STAGE6_CONNECTION	0x00B0
#define STAGE7_CONNECTION	0x00B8
#define STAGE8_CONNECTION	0x00C0
#define STAGE9_CONNECTION	0x00C8
#define STAGE10_CONNECTION	0x00D0
#define STAGE11_CONNECTION	0x00D8


#define STAGE0				0x00E0
#define STAGE0_AMBIENT		0x00F1
#define STAGE0_MAX_AVG		0x00F9
#define STAGE0_UPP_THRES	0x00FA
#define STAGE0_MIN_AVG		0x0100
#define STAGE0_LWR_THRES	0x0101

#define STAGE1				0x0104
#define STAGE1_AMBIENT		0x0115
#define STAGE1_MAX_AVG		0x011D
#define STAGE1_UPP_THRES	0x011E
#define STAGE1_MIN_AVG		0x0124
#define STAGE1_LWR_THRES	0x0125

#define STAGE2				0x0128
#define STAGE2_AMBIENT		0x0139
#define STAGE2_MAX_AVG		0x0141
#define STAGE2_UPP_THRES	0x0142
#define STAGE2_MIN_AVG		0x0148
#define STAGE2_LWR_THRES	0x0149

#define STAGE3				0x014C
#define STAGE3_AMBIENT		0x015D
#define STAGE3_MAX_AVG		0x0165
#define STAGE3_UPP_THRES	0x0166
#define STAGE3_MIN_AVG		0x016C
#define STAGE3_LWR_THRES	0x016D

#define STAGE4				0x0170
#define STAGE4_AMBIENT		0x0181
#define STAGE4_MAX_AVG		0x0189
#define STAGE4_UPP_THRES	0x018A
#define STAGE4_MIN_AVG		0x0190
#define STAGE4_LWR_THRES	0x0191

#define STAGE5				0x0194
#define STAGE5_AMBIENT		0x01A5
#define STAGE5_MAX_AVG		0x01AD
#define STAGE5_UPP_THRES	0x01AE
#define STAGE5_MIN_AVG		0x01B4
#define STAGE5_LWR_THRES	0x01B5

#define STAGE6				0x01B8
#define STAGE6_AMBIENT		0x01C9
#define STAGE6_MAX_AVG		0x01D1
#define STAGE6_UPP_THRES	0x01D2
#define STAGE6_MIN_AVG		0x01D8
#define STAGE6_LWR_THRES	0x01D9

#define STAGE7				0x01DC
#define STAGE7_AMBIENT		0x01ED
#define STAGE7_MAX_AVG		0x01F5
#define STAGE7_UPP_THRES	0x01F6
#define STAGE7_MIN_AVG		0x01FC
#define STAGE7_LWR_THRES	0x01FD

#define STAGE8				0x0200
#define STAGE8_AMBIENT		0x0211
#define STAGE8_MAX_AVG		0x0219
#define STAGE8_UPP_THRES	0x021A
#define STAGE8_MIN_AVG		0x0220
#define STAGE8_LWR_THRES	0x0221

#define STAGE9				0x0224
#define STAGE9_AMBIENT		0x0234
#define STAGE9_MAX_AVG		0x023D
#define STAGE9_UPP_THRES	0x023E
#define STAGE9_MIN_AVG		0x0244
#define STAGE9_LWR_THRES	0x0245

#define STAGE10				0x0248
#define STAGE10_AMBIENT		0x0259
#define STAGE10_MAX_AVG		0x0261
#define STAGE10_UPP_THRES	0x0262
#define STAGE10_MIN_AVG		0x0268
#define STAGE10_LWR_THRES	0x0269

#define STAGE11				0x026C
#define STAGE11_AMBIENT		0x027D
#define STAGE11_MAX_AVG		0x0285
#define STAGE11_UPP_THRES	0x0286
#define STAGE11_MIN_AVG		0x028C
#define STAGE11_LWR_THRES	0x028D
