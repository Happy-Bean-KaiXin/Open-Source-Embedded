/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_motor.h
  * @brief          : Header for bsp_motor.c file.
  ******************************************************************************
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_MOTOR_H
#define __BSP_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

#define PWM_MAX   7200
#define PWM_MIN   -7200

#define RR 30u    //减速比

#define RELOADVALUE_TIM3 __HAL_TIM_GetAutoreload(&htim3)  // 获取自动装载值,本例中为20000
#define COUNTERNUM_TIM3  __HAL_TIM_GetCounter(&htim3)     // 获取编码器定时器中的计数值

#define RELOADVALUE_TIM4 __HAL_TIM_GetAutoreload(&htim4)  // 获取自动装载值,本例中为20000
#define COUNTERNUM_TIM4  __HAL_TIM_GetCounter(&htim4)     // 获取编码器定时器中的计数值

// 电机1
#define IN1(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,(GPIO_PinState)(state))
#define IN2(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,(GPIO_PinState)(state))

// 电机2
#define IN4(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,(GPIO_PinState)(state))
#define IN3(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,(GPIO_PinState)(state))

typedef struct _Motor
{
	int32_t lastAngle;       //上次计数结束时转过的角度
	int32_t totalAngle;      //总共转过的角度
	int16_t loopNum;         //电机计数过零计数
	int speed;             //电机输出轴速度
	float targetSpeed;       //添加设定的目标速度  RPM
	int32_t targetAngle;		 //目标角度(编码器值)
	int pwm;
}Motor;

enum Mode
{
		Speed,
		Angle
};

extern Motor motor1;
extern Motor motor2;

/*! -------------------------------------------------------------------------- */
/*! Public functions list */
void Motor_Init(void);

void PWM_LOAD(int motor1, int motor2);
int Read_Speed(int TIMx);
int gfp_abs(int p);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MOTOR_H */
