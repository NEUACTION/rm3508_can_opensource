/**
  ******************************************************************************
  * @file    comunication 
  * @author  Tmax Sco
  * @version V1.0.0
  * @date    2017.12.31
  * @brief   用于与主控通信，与下级C620电调通信
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "comm.h"
#include "can.h"
#include "ctrl.h"
#include "rm_motor.h"

extern MotorType Motor[8];
extern DriverType Driver[8];

/**
  * @brief  设定电机的运行电流
  * @param 
  * @param 
  * @retval 
  */
void SetCur(float* cur)
{
  uint8_t mbox;	 
	CanTxMsg TxMessage;
	int16_t data[4] = {0};
	
	for(int i = 0; i < 4; i++)
		data[i] = (int16_t)(cur[i]);
	
	TxMessage.StdId=0x200;					     // standard identifier=0
	TxMessage.ExtId=0x200;					     // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			   // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;
					 
 	TxMessage.Data[0] = (data[0]>>8)&0xff;
	TxMessage.Data[1] = data[0]&0xff;
	TxMessage.Data[2] = (data[1]>>8)&0xff;
	TxMessage.Data[3] = data[1]&0xff;
	TxMessage.Data[4] = (data[2]>>8)&0xff;
	TxMessage.Data[5] = data[2]&0xff;
	TxMessage.Data[6] = (data[3]>>8)&0xff;
	TxMessage.Data[7] = data[3]&0xff;
		
	mbox= CAN_Transmit(CAN1, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN1, mbox)!= CAN_TxStatus_Ok));//等待238us
}
	
/**
  * @brief  回应主控请求
  * @attention CAN发送时间较长，此函数一定要放在低优先级任务中
  * @param  
  * @param 
  * @retval 
  */
void CANRespond(void)
{
	UnionDataType txData;

	for(int i = 0; i < 8; i++)
	{
		if(Motor[i].type == NONE)
			break;
		
		switch (Driver[i].command.can_status)
		{
			case 0:
				break;
			case 0x40005856: //VX   读取速度
				txData.data32[0] = 0x00005856;
				txData.data32[1]  = (int32_t)(Driver[i].velCtrl.speed * 1000);
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			case 0x40005149: //IQ	 读取电流
				txData.data32[0] = 0x00005149;
				txData.dataf[1] = Motor[i].cur;
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			case 0x40005850: //PX   读取位置
				txData.data32[0] = 0x00005850;
				txData.data32[1]  = (int32_t)(Driver[i].posCtrl.actualPos);
				CanSendData(Driver[i].command.canId,txData);
				Driver[i].command.can_status = 0;
				break;

			default: break;
		}
	}
}

/**
  * @brief  CanSendData
  * @param 
  * @param 
  * @retval 
  */
void CanSendData(int id,UnionDataType txData)
{
	uint8_t mbox;	 
	CanTxMsg TxMessage;

	TxMessage.StdId= (0x280+id); // standard identifier=0
	TxMessage.ExtId= (0x280+id);					     // extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 // type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			   // the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

	for(int i=0; i<8; i++)
	{
		TxMessage.Data[i] = txData.data8[i];	     // 帧信息 
	}
    		
	mbox= CAN_Transmit(CAN2, &TxMessage);         //1.4us	
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok));//等待238us
}
/************************ (C) COPYRIGHT 2019 ACTION ********************/
