/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sonic.h
  * @brief          : Header for sonic.c file.
  ******************************************************************************
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SONIC_H
#define __SONIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*
ʹ�÷��� ��
	main������ʼ��
	
	Hcsr04Init(&htim2, TIM_CHANNEL_2);  //  ������ģ���ʼ��
	Hcsr04Start();  //  ����������ģ����
	print(&huart1, "hc-sr04 start!\r\n");
	
	while(1) ���� Sonic_Proc();
*/

typedef struct
{
	uint8_t  edge_state;
	uint16_t tim_overflow_counter;
	uint32_t prescaler;
	uint32_t period;
	uint32_t t1;	//	������ʱ��
	uint32_t t2;	//	�½���ʱ��
	uint32_t high_level_us;	//	�ߵ�ƽ����ʱ��
	float    distance;
	TIM_TypeDef* instance;
  uint32_t ic_tim_ch;
	HAL_TIM_ActiveChannel active_channel;
}Hcsr04InfoTypeDef;

extern Hcsr04InfoTypeDef Hcsr04Info;

#define HAL_TIM_MODULE_ENABLED
 
// The Number OF HC-SR04 Sensors To Be Used In The Project
#define HCSR04_UNITS  1
 
typedef struct
{
    GPIO_TypeDef * TRIG_GPIO;
    uint16_t       TRIG_PIN;
    TIM_TypeDef*   TIM_Instance;
    uint32_t       IC_TIM_CH;
    uint32_t       TIM_CLK_MHz;
}HCSR04_CfgType;

extern double distance;

/*! -------------------------------------------------------------------------- */
/*! Public functions list */

void HAL_Delayus(uint32_t nus);

/**
 * @description: ������ģ������벶��ʱ��ͨ����ʼ��
 * @param {TIM_HandleTypeDef} *htim
 * @param {uint32_t} Channel
 * @return {*}
 */
void Hcsr04Init(TIM_HandleTypeDef *htim, uint32_t Channel);

/**
 * @description: HC-SR04����
 * @param {*}
 * @return {*}
 */
void Hcsr04Start();

/**
 * @description: ��ʱ����������жϴ�����
 * @param {*}    main.c���ض���void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
 * @return {*}
 */
void Hcsr04TimOverflowIsr(TIM_HandleTypeDef *htim);

/**
 * @description: ���벶�����ߵ�ƽʱ��->����
 * @param {*}    main.c���ض���void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 * @return {*}
 */
void Hcsr04TimIcIsr(TIM_HandleTypeDef* htim);

/**
 * @description: ��ȡ���� 
 * @param {*}
 * @return {*}
 */
float Hcsr04Read();

void Sonic_Proc(void);


#ifdef __cplusplus
}
#endif

#endif /* __SONIC_H */
