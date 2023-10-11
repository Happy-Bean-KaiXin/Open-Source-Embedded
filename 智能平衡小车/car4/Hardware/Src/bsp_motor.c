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

float Med_Angle = (0.10+2.38)/2.0; // ��е��ֵ - ��ʹС������ƽ��ס�ĽǶ�
//float Med_Angle = 0.0;
int Turn_Out = 0, PWM_Out = 0;

/**************************
��ʱ���жϻص�����
��ڲ�����TIM_HandleTypeDef* htim
***************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if(htim ->Instance == TIM2) {
		motor1.speed = -Read_Speed(3);  // �������԰�װ�������Ҫ����ȡ������
		motor2.speed = +Read_Speed(4);
		
		mpu_dmp_get_data(&pitch, &roll, &yaw);  // ŷ����
		/**
		* ���С������MPU6050��x����ת��Angle��rollֵ������ǣ��� Gyrox�Ƕ�Ӧ�Ľ��ٶ�
		* ���С������MPU6050��y����ת��Angle��pitchֵ������ǣ���Gyroy�Ƕ�Ӧ�Ľ��ٶ�
		**/
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz); // ������
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz); // ���ٶ�
		
		// �������ҵ���ٶ���ȣ���СӲ�����
		if((motor1.speed > 0) && (motor2.speed > 0)) {
			motor1.speed = motor1.speed;
			motor2.speed = motor2.speed;
		}
		else if((motor1.speed < 0) && (motor2.speed < 0)) {
			motor1.speed = motor1.speed;
			motor2.speed = motor2.speed - 1;
		}
		
		// �Ƕȹ���ֹͣ���ת������Ӳ�����Զ���
		if(gfp_abs(pitch)>40)
		{
			HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
			motor2.pwm = 0;
			motor1.pwm = 0;
		}
		else 
		{
			HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
			// 2.������ѹ��ջ�������
			VerTical.Out = Vertical(Med_Angle, pitch, gyroy);  // ֱ����
			VeloCity.Out = Velocity(motor1.speed, motor2.speed);  // �ٶȻ�
			Turn_Out = Turn(gyroz);

			PWM_Out = VerTical.Out + VeloCity.Out;  // ���ĺ�ʽ ֱ�������ٶȻ�

			// 3.�ѿ���������ӵ�����ϣ���ɿ���  ��ת��
			motor1.pwm = PWM_Out - Turn_Out;
			motor2.pwm = PWM_Out + Turn_Out;

			LIMIT(motor1.pwm, PWM_MIN, PWM_MAX);  // �������PWM���
			LIMIT(motor2.pwm, PWM_MIN, PWM_MAX);
		}
		
		PWM_LOAD(motor1.pwm, motor2.pwm);
  }
}
