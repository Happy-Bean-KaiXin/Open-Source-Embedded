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
使用方法 ：
	main函数初始化
	
	Hcsr04Init(&htim2, TIM_CHANNEL_2);  //  超声波模块初始化
	Hcsr04Start();  //  开启超声波模块测距
	print(&huart1, "hc-sr04 start!\r\n");
	
	while(1) 调用 Sonic_Proc();
*/

typedef struct
{
	uint8_t  edge_state;
	uint16_t tim_overflow_counter;
	uint32_t prescaler;
	uint32_t period;
	uint32_t t1;	//	上升沿时间
	uint32_t t2;	//	下降沿时间
	uint32_t high_level_us;	//	高电平持续时间
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
 * @description: 超声波模块的输入捕获定时器通道初始化
 * @param {TIM_HandleTypeDef} *htim
 * @param {uint32_t} Channel
 * @return {*}
 */
void Hcsr04Init(TIM_HandleTypeDef *htim, uint32_t Channel);

/**
 * @description: HC-SR04触发
 * @param {*}
 * @return {*}
 */
void Hcsr04Start();

/**
 * @description: 定时器计数溢出中断处理函数
 * @param {*}    main.c中重定义void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
 * @return {*}
 */
void Hcsr04TimOverflowIsr(TIM_HandleTypeDef *htim);

/**
 * @description: 输入捕获计算高电平时间->距离
 * @param {*}    main.c中重定义void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
 * @return {*}
 */
void Hcsr04TimIcIsr(TIM_HandleTypeDef* htim);

/**
 * @description: 读取距离 
 * @param {*}
 * @return {*}
 */
float Hcsr04Read();

void Sonic_Proc(void);


#ifdef __cplusplus
}
#endif

#endif /* __SONIC_H */
