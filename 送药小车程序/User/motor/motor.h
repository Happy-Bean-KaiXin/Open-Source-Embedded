#ifndef __MOTOR_H
#define __MOTOR_H

#include "tim.h"

/***************关于车子的参数****************/
#define WheelDiameter 0.060f //定义小车轮子直径,6.0cm
#define CarWidth 0.175f		//定义小车两个轮子间距，17.5cm

// 电机1
#define AN1(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,(GPIO_PinState)(state))  //
#define AN2(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,(GPIO_PinState)(state))

// 电机2
#define BN1(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,(GPIO_PinState)(state))
#define BN2(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,(GPIO_PinState)(state))

#define MAX_MOTOR_PWM 4500    // 720*6

//#define Motor_Speed_Max 160   // 160RPM

typedef struct _Motor {
	float speed;
	uint16_t pwm;
	
	long Sigma_MotorPulse;
	int Uinttime_MotorPulse;
	float Distance;
	
	float TargetSpeed;      // RPM
	
	float Target_Position;
	float Num;                // 小车转的圈数
	
} Motor_t;

extern Motor_t Motor1;
extern Motor_t Motor2;

int gfp_abs(int p);
void Motor_Init(void);
void Set_PWM(int motor_lpwm, int motor_rpwm);
void Motor_Stop(void);

#endif
