#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"
#include "struct_typedef.h"
//tim4_parameter
#define TIM4_CLOCK (2*HAL_RCC_GetPCLK1Freq())    //Tclk/(arr+1)=84000000/(10000)
#define TIME4_ARR (9999)
#define TIM4_GAIN (TIM4_CLOCK/(TIME4_ARR+1))
#define VOLUME(x) (((x)*(TIME4_ARR+1)/100)-1) //x=1-100

//frequency
#define L1 ((TIM4_GAIN/262)-1)//低调　do 的频率
#define L2 ((TIM4_GAIN/296)-1)//低调　re 的频率
#define L3 ((TIM4_GAIN/330)-1)//低调　mi 的频率
#define L4 ((TIM4_GAIN/349)-1)//低调　fa 的频率
#define L5 ((TIM4_GAIN/392)-1)//低调　sol 的频率
#define L6 ((TIM4_GAIN/440)-1)//低调　la 的频率
#define L7 ((TIM4_GAIN/494)-1)//低调　si 的频率

#define M1 ((TIM4_GAIN/523)-1)//中调　do 的频率
#define M2 ((TIM4_GAIN/587)-1)//中调　re 的频率
#define M3 ((TIM4_GAIN/659)-1)//中调　mi 的频率
#define M4 ((TIM4_GAIN/699)-1)//中调　fa 的频率
#define M5 ((TIM4_GAIN/784)-1)//中调　sol的频率
#define M6 ((TIM4_GAIN/880)-1)//中调　la 的频率
#define M7 ((TIM4_GAIN/988)-1)//中调　si 的频率

#define H1 ((TIM4_GAIN/1048)-1)//高调　do 的频率
#define H2 ((TIM4_GAIN/1176)-1)//高调　re 的频率
#define H3 ((TIM4_GAIN/1320)-1)//高调　mi 的频率
#define H4 ((TIM4_GAIN/1480)-1)//高调　fa 的频率
#define H5 ((TIM4_GAIN/1640)-1)//高调　sol的频率
#define H6 ((TIM4_GAIN/1760)-1)//高调　la 的频率
#define H7 ((TIM4_GAIN/1976)-1)//高调　si 的频率


extern void buzzer_on(uint16_t frequency, uint16_t volume);

extern void buzzer_off(void);

#endif
