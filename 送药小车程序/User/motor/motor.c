#include "motor.h"

Motor_t Motor1;
Motor_t Motor2;

void Motor_Init(void) {	
	HAL_TIM_Base_Start_IT(&htim4);                // 开启10ms定时器中断
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);     // pwm 定时器1通道1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);     // pwm 定时器1通道4
}

/**************************
绝对值函数
入口参数：int p
取入口参数的绝对值
***************************/
int gfp_abs(int p) {
	int q;
	q = p > 0? p : (-p);
	return q;
}

/**
* @brief:设置电机速度PWM值
* @param:
* 			[in]motor_rpwm: 左电机PWM PWM占空比,范围-4500到4500，绝对值值越大速度越快，大于0正向转，小于0逆向转
* 			[in]motor_b: 右电机PWM PWM占空比,范围-4500到4500，绝对值值越大速度越快，大于0正向转，小于0逆向转
* @return:None
*/
void Set_PWM(int motor_lpwm, int motor_rpwm) {
	
	if(motor_lpwm > 0) { 
		AN1(0); AN2(1); 
	}
	else if(motor_lpwm < 0) { 
		AN1(1); AN2(0); 
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, gfp_abs(motor_lpwm));  // 左电机
	
	if(motor_rpwm > 0) { 
		BN1(0); BN2(1); 
	}
	else if(motor_rpwm < 0) { 
		BN1(1); BN2(0); 
	}
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, gfp_abs(motor_rpwm));  // 右电机
}

void Motor_Stop(void) {
	AN1(0); AN2(0); BN1(0); BN2(0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

	
