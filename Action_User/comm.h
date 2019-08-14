/**
  ******************************************************************************
  * @file    comm.h
  * @author  Tmax Sco
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __comm_h
#define __comm_h

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
/* Exported types ------------------------------------------------------------*/
/** 
  * @brief  联合体、共用体数据类型  
  * @note     
  */
typedef union
{
	
	uint8_t data8[8];	
	int16_t data16[4];	
	int data32[2];
	float dataf[2];
	
}UnionDataType;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SetCur(float* cur);
void CanSendData(int id,UnionDataType txData);
void CANRespond(void);

#endif

/****************** (C) COPYRIGHT 2017 ACTION *****END OF FILE*************/

