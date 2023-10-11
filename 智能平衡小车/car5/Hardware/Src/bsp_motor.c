#include "bsp_motor.h"
#include "tim.h"
#include "pid.h"
#include "usart.h"
#include "sonic.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

/**************Private variables*******************/
Motor motor1;
Motor motor2;
enum Mode mode = Speed;
/**************�ڲ���������*******************/
void Motor_Send(enum Mode mode);

/**
 * @function: float Motor_Cnt_PWM(int encoder_cnt)
 * @description: ͨ�����ת�ٻ�ȡ�����pwm
 * @param {int encoder_cnt} ����������ֵ 
 * @return {Motor_PWM} ���PWM
 */
float Motor_Cnt_PWM(int encoder_cnt) {
	float Motor_PWM;
	
	Motor_PWM = (10/8.0) * (encoder_cnt - 5) + 10;    // ֱ�ߵĵ�бʽ����
  return Motor_PWM;
}

void Motor_Init(void)
{
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // ����������A   A��PB4
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // ����������B;  B��PB5	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // ����������A   A��PB6
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // ����������B;  B��PB7	
	
	HAL_TIM_Base_Start_IT(&htim2);                // ����10ms��ʱ���ж�
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // pwm ��ʱ��1ͨ��1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);     // pwm ��ʱ��1ͨ��4
	
	__HAL_TIM_SET_COUNTER(&htim3, 0); 
	__HAL_TIM_SET_COUNTER(&htim4, 0); 
}

void Motor_Send(enum Mode mode)
{
	if(mode == Speed)  //����PID���㣬���������趨��Ŀ��ֵtargetSpeed
	{
	}
	else if(mode == Angle)
	{
	}
	
}

/**********************
�������ٶȶ�ȡ����
��ڲ�������ʱ�� int TIMx
�ٶ���λ�Ƶ���  dx/dt = ��x/��t
**********************/
int Read_Speed(int TIMx)
{
  int value_1;
  switch(TIMx)
  {
    case 3:
			value_1 = (short)COUNTERNUM_TIM3;  // ��ȡ������������
			__HAL_TIM_SET_COUNTER(&htim3, 0);  // ������ֵ����  �루__HAL_TIM_SetCounter�� ����������һ����
			break;
    //IF�Ƕ�ʱ��2��1.�ɼ��������ļ���ֵ�����档2.����ʱ���ļ���ֵ���㡣
    case 4:
			value_1 = (short)COUNTERNUM_TIM4;
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		break;
    default:value_1 = 0;
  }
  return value_1;
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

/**************************
��ֵ����
��ڲ�����int motor1, int motor2
PID������������pwmֵ����˺����С�
***************************/
void PWM_LOAD(int motor1, int motor2) {
	if(motor1 > 0){IN1(0); IN2(1);}
	else 					{IN1(1); IN2(0);}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, gfp_abs(motor1));
	
	if(motor2 > 0){IN3(0); IN4(1);}
	else 					{IN3(1); IN4(0);}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, gfp_abs(motor2));
}

/**************************
��ʱ���жϻص�����
��ڲ�����TIM_HandleTypeDef* htim
***************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if(htim ->Instance == TIM2) {
//		mpu_dmp_get_data(&pitch, &roll, &yaw);  // ŷ����

  }
}
