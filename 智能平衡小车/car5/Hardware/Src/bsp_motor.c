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
/**************内部函数声明*******************/
void Motor_Send(enum Mode mode);

/**
 * @function: float Motor_Cnt_PWM(int encoder_cnt)
 * @description: 通过电机转速获取电机的pwm
 * @param {int encoder_cnt} 编码器计数值 
 * @return {Motor_PWM} 电机PWM
 */
float Motor_Cnt_PWM(int encoder_cnt) {
	float Motor_PWM;
	
	Motor_PWM = (10/8.0) * (encoder_cnt - 5) + 10;    // 直线的点斜式方程
  return Motor_PWM;
}

void Motor_Init(void)
{
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // 开启编码器A   A相PB4
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // 开启编码器B;  B相PB5	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1); // 开启编码器A   A相PB6
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // 开启编码器B;  B相PB7	
	
	HAL_TIM_Base_Start_IT(&htim2);                // 开启10ms定时器中断
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // pwm 定时器1通道1
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);     // pwm 定时器1通道4
	
	__HAL_TIM_SET_COUNTER(&htim3, 0); 
	__HAL_TIM_SET_COUNTER(&htim4, 0); 
}

void Motor_Send(enum Mode mode)
{
	if(mode == Speed)  //进行PID计算，传入我们设定的目标值targetSpeed
	{
	}
	else if(mode == Angle)
	{
	}
	
}

/**********************
编码器速度读取函数
入口参数：定时器 int TIMx
速度是位移的求导  dx/dt = Δx/Δt
**********************/
int Read_Speed(int TIMx)
{
  int value_1;
  switch(TIMx)
  {
    case 3:
			value_1 = (short)COUNTERNUM_TIM3;  // 获取编码器脉冲数
			__HAL_TIM_SET_COUNTER(&htim3, 0);  // 将计数值清零  与（__HAL_TIM_SetCounter） 这俩函数是一样的
			break;
    //IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
    case 4:
			value_1 = (short)COUNTERNUM_TIM4;
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		break;
    default:value_1 = 0;
  }
  return value_1;
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

/**************************
赋值函数
入口参数：int motor1, int motor2
PID运算完后的最终pwm值读入此函数中。
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
定时器中断回调函数
入口参数：TIM_HandleTypeDef* htim
***************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
  if(htim ->Instance == TIM2) {
//		mpu_dmp_get_data(&pitch, &roll, &yaw);  // 欧拉角

  }
}
