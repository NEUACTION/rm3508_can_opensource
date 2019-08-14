#ifndef __timer_h
#define __timer_h

#include "stm32f4xx_tim.h"
//宏延时函数
#define  SYSCLK        168         //指明MCU工作频率为168MHz
#define  A             3           //一次循环所花的周期数
#define  B             3           //调用、初始化、返回总共所用的周期数
#define  delay_us(nus)   wait(((nus)*(SYSCLK)-(B))/(A))
#define  delay_ms(nms)   delay_us((nms)*1000)
#define  delay_s(ns)     delay_ms((ns)*1000)
//精确延时函数调用wait
void  wait(uint32_t n);

void TIM_Init(TIM_TypeDef * TIMx, uint16_t arr, uint16_t psr,uint16_t prepri,uint16_t subpri); 
void TIM_Delayms(TIM_TypeDef * TIMx, uint32_t DelayMs);
void TIM_Delayus(TIM_TypeDef * TIMx, uint16_t Delayus);
void TIM_Delay100us(TIM_TypeDef * TIMx, uint16_t Delay100us);

void TIM3_Pwm_Init(u32 arr,u32 psc);
void TIM4_Pwm_Init(u32 arr,u32 psc);

#endif



