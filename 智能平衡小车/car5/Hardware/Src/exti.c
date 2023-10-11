#include "exti.h"
#include "bsp_motor.h"
#include "pid.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


float Med_Angle = (0.10+2.38)/2.0; // ��е��ֵ - ��ʹС������ƽ��ס�ĽǶ�
//float Med_Angle = 0.0;
int Turn_Out = 0, PWM_Out = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {  
    // �����жϴ�����
		
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

			PWM_Out = VerTical.Out + VeloCity.Out;  // ���ĺ�ʽ

			// 3.�ѿ���������ӵ�����ϣ���ɿ���
			motor1.pwm = PWM_Out - Turn_Out;
			motor2.pwm = PWM_Out + Turn_Out;

			LIMIT(motor1.pwm, PWM_MIN, PWM_MAX);  // �������PWM���
			LIMIT(motor2.pwm, PWM_MIN, PWM_MAX);
		}
		
		PWM_LOAD(motor1.pwm, motor2.pwm);
    
  }
}
