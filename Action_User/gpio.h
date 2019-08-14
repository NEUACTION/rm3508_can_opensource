#ifndef __GPIO_H

#define __GPIO_H

#include "stm32f4xx_gpio.h"


#define LIGHT_UNLASHFAN   GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_7)
#define LIGHT_GRABPOST    GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define LIGHT_GRABFAN  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)
#define LIGHT_HIGH     GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_10)
#define SWITCH_END_0    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)
#define SWITCH_END_1    GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_12)

#define BEEP_ON          GPIO_SetBits(GPIOE,GPIO_Pin_2)
#define BEEP_OFF         GPIO_ResetBits(GPIOE,GPIO_Pin_2)
 
void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
	
#endif
