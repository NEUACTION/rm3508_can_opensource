#include "stm32f4xx_can.h"

#define CAN_ENABLE 1

void CAN_Config(CAN_TypeDef* CANx, 
				uint32_t CAN_BaudRate,
				GPIO_TypeDef * GPIOx,
				uint16_t CAN_RxPin,
				uint16_t CAN_TxPin);

uint8_t CAN_RxMsg(CAN_TypeDef* CANx,
				  uint32_t * StdId,
				  uint8_t * buf,
				  uint8_t len);

uint8_t CAN_TxMsg(CAN_TypeDef* CANx,
				  uint32_t StdId,
				  uint8_t * buf,
				  uint8_t len);


