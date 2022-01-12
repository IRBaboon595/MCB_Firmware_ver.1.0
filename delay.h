/************************************************
*Author: Alexander Plotnikov
*Data: 04.2018
*Name: delay.h
*Type: Delay header file
*************************************************/

#include "stdint.h"







void DWT_Init(void);
static __inline uint32_t delta(uint32_t t0, uint32_t t1);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
