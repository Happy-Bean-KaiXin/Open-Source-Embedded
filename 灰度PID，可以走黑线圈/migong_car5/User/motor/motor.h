#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "tim.h"

/***************���ڳ��ӵĲ���****************/
#define WheelDiameter 0.060f //����С������ֱ��,5.7cm
#define CarWidth 19.4f		//����С���������Ӽ�࣬19.4cm

// ���1
#define IN1(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,(GPIO_PinState)(state))  // 0 1��ת  1 0 ��ת
#define IN2(state) HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,(GPIO_PinState)(state))
// ���2
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
