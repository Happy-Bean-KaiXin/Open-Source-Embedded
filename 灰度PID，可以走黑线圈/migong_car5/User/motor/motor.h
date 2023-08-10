#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "tim.h"

/***************关于车子的参数****************/
#define WheelDiameter 0.060f //定义小车轮子直径,5.7cm
#define CarWidth 19.4f		//定义小车两个轮子间距，19.4cm

// 电机1
#define IN1(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,(GPIO_PinState)(state))  // 0 1正转  1 0 反转
#define IN2(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,(GPIO_PinState)(state))
// 电机2
#define IN4(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,(GPIO_PinState)(state))
#define IN3(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,(GPIO_PinState)(state))

#define MAX_MOTOR_PWM 4500    // 720*6

//#define Motor_Speed_Max 160   // 160RPM

typedef struct _Motor {
	float speed;
	float spin_speed;
	uint16_t pwm;
	
	long Sigma_MotorPulse;
	int Uinttime_MotorPulse;
	float Distance;
	
	float targetSpeed;      //RPM
	
} Motor_t;

extern Motor_t Motor1;
extern Motor_t Motor2;
extern Motor_t Motor;

int gfp_abs(int p);
void Motor_Init(void);
void Set_PWM(int motor_lpwm, int motor_rpwm);
void Motor_Stop(void);

#endif
