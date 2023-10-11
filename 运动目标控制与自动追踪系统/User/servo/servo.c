#include "servo.h"

float Init_CompareX;//舵机占空比初始值
float Init_CompareY;//舵机占空比初始值

float Angle_Flag_X=0.0;
float Angle_Flag_Y=0.0;

/*
 * 舵机初始化 初始角度
 *
 */
void Servo_Angle_Init(void)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);		// 使能PWM输出  A15  x轴
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);		// 使能PWM输出  B3   y轴
}

/* X轴舵机初始化微调
 * Angle:调整角度
 * Direction:调整方向
 */
void Servo_X_Angle_Set(float Angle)
{
    Init_CompareX =50+((((-1)*Angle+95)*(1/90.0))/20*TIM2_ARR);//90度补偿，50ms补偿
    Angle_Flag_X=Angle;//当前角度
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Init_CompareX);
}
/* Y轴舵机初始化微调
 * Angle:调整角度
 * Direction:调整方向
 */
void Servo_Y_Angle_Set(float Angle)
{
    Init_CompareY =50+((((-1)*Angle+95)*(1/90.0))/20*TIM2_ARR);//90度补偿，50ms补偿
    Angle_Flag_Y=Angle;//当前角度
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Init_CompareY);  // 设置舵机转动PWM
}

/**************************************************************************
函数功能：位置式PID控制器
入口参数：position 当前舵机转动位置的定时器比较值
         target    给定舵机旋转的角度
返回  值：电机PWM
根据位置式离散PID公式 
pwm=Kp*e(k)+Ki*∑e(k)+Kd[e（k）-e(k-1)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  
∑e(k)代表e(k)以及之前的偏差的累积和;其中k为1,2,,k;
pwm代表输出
**************************************************************************/
int Servo_Position_PIDY(int position, int target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias = target-position;                                  // 计算偏差
	 Integral_bias += Bias;	                                    // 求出偏差的积分
	 LIMIT(Integral_bias, -5000, 5000);                       // 积分限幅
	 Pwm = (Servo_Position.Kp/100)*Bias+(Servo_Position.Ki/100)*Integral_bias+(Servo_Position.Kd/100)*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                          // 保存上一次偏差 
	 return Pwm;                                              // 增量输出
}

int Servo_Position_PIDX(int position, int target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias = target-position;                                  // 计算偏差
	 Integral_bias += Bias;	                                    // 求出偏差的积分
	 LIMIT(Integral_bias, -5000, 5000);                       // 积分限幅
	 Pwm = (Servo_Position.Kp/100)*Bias+(Servo_Position.Ki/100)*Integral_bias+(Servo_Position.Kd/100)*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                          // 保存上一次偏差 
	 return Pwm;                                              // 增量输出
}

// 传入角度返回，返回定时器比较值
int Servo_Angle_Compare(float Angle) {
   return  50+(((-Angle+95)*(1/90.0))/20*TIM2_ARR);//90度补偿，50ms补偿
}

// 传入定时器的比较值，传出舵机输出角度
float Servo_Compare_Angle(uint32_t Compare) {
    return -(((Compare-50.0)*20*90.0)/TIM2_ARR - 95);
}

void Set_Pwm_ServoX(int motox) {
	int Position;
	Position += motox;
	
	if(Position>1250) Position=1250;
	if(Position<250) Position=250;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Position);  // 设置舵机转动PWM
}

void Set_Pwm_ServoY(int motoY) {
	int Position;
	Position += motoY;
	
	if(Position>1250)Position=1250;
	if(Position<250)Position=250;
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Position);  // 设置舵机转动PWM
}
/*********************************************END OF FILE*****************************************/






