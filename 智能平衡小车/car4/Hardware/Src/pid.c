#include "pid.h"
#include "usart.h"
#include "tim.h"
#include "bsp_oled.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mpu6050.h"


PIDTypedef VerTical;
PIDTypedef VeloCity;

extern uint8_t fx;

/**
负反馈的PD直立环 + 正反馈的PI速度环。
直立环为主
速度环为辅
转向环
*/

/**
 * @function: int Vertical(float Med, float Angle, float gyro_y) 
 * @description: 直立环PD控制
 * 如果你的电机PWM=7200为满幅，即占空比100%。
 * 如果kp=720，角度偏差10度，电机就到达满转了。
 * 我们不会容忍小车偏差10度以上，所以kp的范围（0 - 720），但是具体情况具体分析
 * 在直立环中，PD控制入口参数为小车的姿态角以及姿态角对应的角速度
 * @param1 float Med, 期望角度
 * @param2 float Angle, 真实角度
 * @return 直立环输出
 */
int Vertical(float Med, float Angle, float gyro_y) {
	int PWM_out;
	
	PWM_out = VerTical.Kp * (Angle - Med) + VerTical.Kd * (gyro_y - 0);
	
	return PWM_out;
}

/**
 * @function: 
 * @description: 速度环PI控制
 * @param1 int Moto_Left, 左电机速度
 * @param2 int Moto_Right 右电机速度
 * @return 
 */
int Velocity(int Moto_Left, int Moto_Right) {
	static int PWM_out, ENco_Err_LowOut, ENco_Err_LowOut_Last,\
		Integral, Error;
	
	float a = 0.7; 
	
	// 1.计算速度偏差
	Error = (Moto_Left + Moto_Right) - 0;
	// 2.对速度进行低通滤波 low_out = (1-a)*Ek + a*low_out_last
	ENco_Err_LowOut = (1-a)*Error + a*ENco_Err_LowOut_Last;
	ENco_Err_LowOut_Last = ENco_Err_LowOut;
	// 3.对速度偏差进行积分位移
	Integral += ENco_Err_LowOut;
	LIMIT(Integral, -10000, 10000);
	
	// 4.电机关闭后清除积分
	if(pitch < -40 || pitch > 40)
		Integral = 0;
	
	// 4.对速度环进行输出
	PWM_out = (int)(VeloCity.Kp * ENco_Err_LowOut + VeloCity.Ki * Integral);
	
	return PWM_out;
}

/**
 * @function: 
 * @description: 转向环*z轴角速度
 * @param: 
 * @return 
 */
int Turn(int gyro_z) {
	int PWM_out;
	
	PWM_out = (-0.25)*gyro_z;
	
	return PWM_out;
}

void print(UART_HandleTypeDef* huart, const char* buf, ...) {
  const char *p = buf;
  char str[255] = {0};
  va_list v;
  va_start(v, buf);
  vsprintf(str, buf, v); //使用可变参数的字符串打印。类似sprintf
  HAL_UART_Transmit(huart, (uint8_t* )str, strlen(str), 0xFF);  // 阻塞式发送
  va_end(v);
}

//初始化pid参数
void PID_Init(void) {
	// 直立环PD  正反馈调节
	VerTical.Kp = 300.0f;//400
	VerTical.Kd = 1.3;
	
	// 速度环PI  Kp与Ki是线性关系 Ki = （1/200）* Kp
	VeloCity.Kp = -59.0f;  // 60
	VeloCity.Ki = VeloCity.Kp/299.0;
	
}

unsigned char Key_Scan(void)  //带有返回值的按键扫描函数
{
	static unsigned char ucKey_value;	//定义一个按键值的变量
	
	if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == GPIO_PIN_RESET)	//按键B1按下键值返回为1
		ucKey_value=1;
	else if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == GPIO_PIN_RESET)	//按键B2按下键值返回为2
		ucKey_value=3;
	else
		ucKey_value=0;	//无按键按下键值返回为0
	
	return ucKey_value;
}

void Key_Proc(void)	//独立按键处理函数对按键的简单控制
{
	/*沿用uwTick的变量定义方式定义滴答定时器按键打点变量*/
	static __IO uint32_t uwTick_KEY_Speed;

	unsigned char ucKey_value,ucKey_Dwon,ucKey_Up;
	static unsigned char ucKey_Old = 0; //静态变量上一次的键值
	
	//利用滴答计时器定义按键扫描时间100ms
	if(uwTick-uwTick_KEY_Speed < 50) return;
	uwTick_KEY_Speed = uwTick;
	
	ucKey_value = Key_Scan();
	ucKey_Dwon = ucKey_value & ( ucKey_value ^ ucKey_Old );	//按下为按键值，其它为0
	ucKey_Up = ~ucKey_value & ( ucKey_value ^ ucKey_Old );	//松手为抬起前的按键值，其他为0 
	ucKey_Old = ucKey_value;
	
	switch(ucKey_Dwon) {
		case 1: fx += 10; OLED_Clear(); break;
		case 3: fx -= 10; OLED_Clear(); break;
	}
}


/**
 * @function: int GetMotorPulse(TIM_HandleTypeDef* htim)
 * @description: 使用STM32编码器模式，读取编码器产生的脉冲值
 * @param {*} 
 * @return {*}
 */
int GetMotorPulse(TIM_HandleTypeDef* htimx)
{
	static short encoder_counter = 0;//STM32编码器模式读取的总脉冲数
	
	encoder_counter = (short) __HAL_TIM_GET_COUNTER(htimx); // 获取编码器数值
	__HAL_TIM_SET_COUNTER(&htim3,0);                         // 计数器清零
	
	return encoder_counter;
}
