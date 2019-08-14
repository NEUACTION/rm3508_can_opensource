/**
  ******************************************************************************
  * @file     
  * @author  lxy
  * @version 
  * @date    
  * @brief   
  ******************************************************************************
  * @attention
  *
  *
  *`
  * 
  ******************************************************************************
  */ 
/* Includes -------------------------------------------------------------------*/
#include "flash.h"
#include "stm32f4xx_flash.h"
#include "string.h"
#include "config.h"
/* Private  typedef -----------------------------------------------------------*/
/* Private  define ------------------------------------------------------------*/
#define FLASH_USER_ADDRESS 0x08040000   //FLASH起始地址
/* Private  macro -------------------------------------------------------------*/
/* Private  variables ---------------------------------------------------------*/
static uint8_t  flashdata[POOLSIZE*4];  //从flash中取出的数据
static int    *Result;
static uint8_t  flag=0;

/* Exported function prototypes -----------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
/**
  * @brief  向FLASH中写入数据
	*
  * @param  data :  存储数据的指针
  * @param  len  :  写入的数据量，单位：byte
  * @retval None
  */
void Flash_Write(uint8_t *data,uint32_t len)
{
  uint32_t count;
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_OPERR|FLASH_FLAG_WRPERR|  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
	FLASH_EraseSector(FLASH_Sector_6,VoltageRange_3);
	FLASH_WaitForLastOperation();
	for(count=0;count<len;count++)
	{
		FLASH_ProgramByte(FLASH_USER_ADDRESS+count,*(data+count));
	}
	FLASH_Lock();
}

/**
  * @brief  从FLASH中读取数据
	*
  * @param  data :  存储数据的指针
  * @param  len  :  读取的数据量，单位：byte
  * @retval None
  */
void Flash_Read(uint8_t *data,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
	//一个字节一个字节地进行赋值
	  *(data+i)= *((uint8_t *)(FLASH_USER_ADDRESS+i));
}
/**
  * @brief  初始化FLASH
	*
  * @param  None
  * @retval None
  */
void Flash_Init(void)
{
	
	Flash_Read(flashdata,POOLSIZE*4);  
	
	Result=(int *)flashdata;
	
}
/**
  * @brief  获取从FLASH里得到的数据的指针
	*
  * @param  None
  * @retval flashdata : 指向flash数据的指针
  */
uint8_t *GetFlashArr(void)
{
	return flashdata;
}
/**
  * @brief  从FLASH里得到的数据分两个部分，第一部分为数据区，第二部分为
  *         计数区，该函数用于获得数据区的指针
  * @param  None
  * @retval Result : 指向flash数据区的指针
  */
int*GetResultArr(void)
{
	return Result;
}

/**
  * @brief  获得FLASH是否需要更新的标志位
  *         
  * @param  None
  * @retval flag : flash更新的标志位
  */
uint8_t GetFlashUpdataFlag(void)
{
	return flag;
}
/**
  * @brief  设置FLASH更新标志位的值
  *         
  * @param  val  : 赋给更新标志位的值
  * @retval None
  */
void  SetFlashUpdateFlag(uint8_t val)
{
	flag=val;
}
/************************ (C) COPYRIGHT 2016 ACTION *****END OF FILE****/

