#include "exti.h"
#include "bsp_motor.h"
#include "pid.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


float Med_Angle = (0.10+2.38)/2.0; // 机械中值 - 能使小车真正平衡住的角度
//float Med_Angle = 0.0;
int Turn_Out = 0, PWM_Out = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0)
  {  
    // 调用中断处理函数
		
		motor1.speed = -Read_Speed(3);  // 电机是相对安装，因此需要进行取反处理
		motor2.speed = +Read_Speed(4);
		mpu_dmp_get_data(&pitch, &roll, &yaw);  // 欧拉角
		/**
		* 如果小车是绕MPU6050的x轴旋转，Angle是roll值（横滚角）， Gyrox是对应的角速度
		* 如果小车是绕MPU6050的y轴旋转，Angle是pitch值（横滚角），Gyroy是对应的角速度
		**/
		MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz); // 陀螺仪
		MPU_Get_Accelerometer(&aacx, &aacy, &aacz); // 加速度
		
		// 限制左右电机速度相等，减小硬件误差
		if((motor1.speed > 0) && (motor2.speed > 0)) {
			motor1.speed = motor1.speed;
			motor2.speed = motor2.speed;
		}
		else if((motor1.speed < 0) && (motor2.speed < 0)) {
			motor1.speed = motor1.speed;
			motor2.speed = motor2.speed - 1;
		}
		
		// 角度过大，停止电机转动，视硬件特性而定
		if(gfp_abs(pitch)>40)
		{
			HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
			motor2.pwm = 0;
			motor1.pwm = 0;
		}
		else 
		{
			HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
			// 2.将数据压入闭环控制中
			VerTical.Out = Vertical(Med_Angle, pitch, gyroy);  // 直立环
			VeloCity.Out = Velocity(motor1.speed, motor2.speed);  // 速度环
			Turn_Out = Turn(gyroz);

			PWM_Out = VerTical.Out + VeloCity.Out;  // 更改后公式

			// 3.把控制输出量加到电机上，完成控制
			motor1.pwm = PWM_Out - Turn_Out;
			motor2.pwm = PWM_Out + Turn_Out;

			LIMIT(motor1.pwm, PWM_MIN, PWM_MAX);  // 限制最大PWM输出
			LIMIT(motor2.pwm, PWM_MIN, PWM_MAX);
		}
		
		PWM_LOAD(motor1.pwm, motor2.pwm);
    
  }
}
