#include "motor.h"
#include "encoder.h"

Motor_t Motor1;
Motor_t Motor2;
Motor_t Motor;

void Motor_Init(void) {	
	HAL_TIM_Base_Start_IT(&htim4);                // ����10ms��ʱ���ж�
	
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);     // pwm ��ʱ��1ͨ��1
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);     // pwm ��ʱ��1ͨ��4
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 720*10);    //�޸ıȽ�ֵ���޸�ռ�ձ�
//	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, 720*10);    //�޸ıȽ�ֵ���޸�ռ�ձ�
	Set_PWM(-720*0, -720*0);
}

/**************************
����ֵ����
��ڲ�����int p
ȡ��ڲ����ľ���ֵ
***************************/
int gfp_abs(int p) {
	int q;
	q = p > 0? p : (-p);
	return q;
}

/**
* @brief:���õ���ٶ�PWMֵ
* @param:
* 			[in]motor_rpwm: ����PWM PWMռ�ձ�,��Χ-4500��4500������ֵֵԽ���ٶ�Խ�죬����0����ת��С��0����ת
* 			[in]motor_b: �ҵ��PWM PWMռ�ձ�,��Χ-4500��4500������ֵֵԽ���ٶ�Խ�죬����0����ת��С��0����ת
* @return:None
*/
void Set_PWM(int motor_lpwm, int motor_rpwm) {
	if(motor_lpwm > 0) { IN1(1); IN2(0); }
	else               { IN1(0); IN2(1); }
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, gfp_abs(motor_lpwm));
	if(motor_rpwm > 0) { IN3(0); IN4(1); }
	else               { IN3(1); IN4(0); }
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, gfp_abs(motor_rpwm));
}

void Motor_Stop(void) {
	IN1(0); IN2(0); IN3(0); IN4(0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

	
