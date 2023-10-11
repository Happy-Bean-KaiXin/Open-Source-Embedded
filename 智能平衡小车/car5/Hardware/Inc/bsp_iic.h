/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_iic.h
  * @brief          : Header for bsp_iic.c file.
  ******************************************************************************
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "main.h"

#define IIC_WR	0		
#define IIC_RD	1		

/*! -------------------------------------------------------------------------- */
/*! Public functions list */
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(uint8_t _ucByte);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
uint8_t IIC_CheckDevice(uint8_t _Address);
void IIC_GPIO_Init(void);

void EEPROM_write(unsigned char address,unsigned char data);
unsigned char EEPROM_read(unsigned char address);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IIC_H */
