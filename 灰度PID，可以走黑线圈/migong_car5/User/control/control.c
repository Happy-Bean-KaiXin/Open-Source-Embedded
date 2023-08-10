#include "control.h"
#include "stdio.h"


#define pi 3.1415926
float WheelOneCircleDis=WheelDiameter*pi;//定义小车轮子行走一圈距离变量

// 定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM4){
		// 获取当前单位时间脉冲
		Motor1.Uinttime_MotorPulse = -GetMotorPulse(1);  
		Motor2.Uinttime_MotorPulse = -GetMotorPulse(3);
		
		// 更新累计脉冲数
		Motor1.Sigma_MotorPulse += Motor1.Uinttime_MotorPulse;
		Motor2.Sigma_MotorPulse += Motor2.Uinttime_MotorPulse;
		
		// 更新小车当前行驶距离   总脉冲数/每一圈的脉冲数 * 周长  == 距离
		Motor1.Distance = Motor1.Sigma_MotorPulse/ACircleEncoder * pi * WheelDiameter;
		Motor2.Distance = Motor2.Sigma_MotorPulse/ACircleEncoder * pi * WheelDiameter;
		
		// 将读取到的脉冲转换为rpm
//		Motor1.speed = ((float)Motor1.Uinttime_MotorPulse)/ACircleEncoder * 6000.0f;   // 将10ms转换为分需要乘以6000  其中ACircleEncoder为宏定义 1560（编码器一圈所需的脉冲数）
//		Motor2.speed = ((float)Motor2.Uinttime_MotorPulse)/ACircleEncoder * 6000.0f;   // 将10ms转换为分需要乘以6000
		
		// 将读取到脉冲转换为m/s
		Motor1.speed = (float)Motor1.Uinttime_MotorPulse/ACircleEncoder * pi * WheelDiameter * 100;
		Motor2.speed = (float)Motor2.Uinttime_MotorPulse/ACircleEncoder * pi * WheelDiameter * 100;
	}
}
	
void OLED_Proc(void) {
	
	static __IO uint32_t uwTick_OLED_Speed;
	char buf[22];
	
	if(uwTick - uwTick_OLED_Speed < 100) return;
	uwTick_OLED_Speed = uwTick;
	
	sprintf(buf, "1 = %d ", Get_Adc(0));
	OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
	sprintf(buf, "2 = %d ",Get_Adc(1));
	OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
	sprintf(buf, "3 = %d ", Get_Adc(4));
	OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
	sprintf(buf, "4 = %d ", Get_Adc(5));
	OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
	sprintf(buf, "5 = %d ", Get_Adc(8));
	OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
}

/**
 *@brief:根据循迹传感器pid调节小车转向使小车处于黑线中间
 * @param:
 *        [in]int TraceDate: 循迹传感器输出的值
 * @return: 返回调节电机速度的转向pwm
 */
int ChangeTraceTurn(int TraceDate)
{
	int pwm=0;
	int bias;
	bias = TraceDate;
	pwm = Turn_PID_Realize( bias);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:根据pid调节左边电机到目标速度
 * @param:
 *        [in]int EncodeSpdL: 当前左电机编码器测速值
 *        [in]float TarSpdL:左边电机目标速度,最大速度越1.19m/s
 * @return: 返回左边电机计算后的pwm占空比
 */
int ChangeSpeedMotorL(int NowEncodeSpdL,float TarSpdL)
{
	int pwm=0;
	int TarEncodeSpdL;
	TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100));//根据目标速度求出目标编码器速度

	pwm=Position_PID_Realize(NowEncodeSpdL, TarEncodeSpdL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:根据pid调节右边电机到目标速度
 * @param:
 *        [in]int NowEncodeSpdR: 当前右电机编码器测速值
 *        [in]float TarSpdR:右边电机目标速度,根据限幅最大速度约为0.7m/s
 * @return: 返回右边电机计算后的pwm占空比
 */
int ChangeSpeedMotorR(int NowEncodeSpdR,float TarSpdR)
{
	int pwm=0;
	int TarEncodeSpdR;
	TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100));//根据目标速度求出目标编码器速度
	
	pwm=Position_PID_Realize(NowEncodeSpdR, TarEncodeSpdR);
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//限幅
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	return pwm;
}

/**
 *@brief: 让小车根据循迹黑线走
 *@param:
 *        [in]TraceDate: 循迹传感器输出的值
 *        [in]TarSpeed:循迹的目标速度,速度大小0-0.7m/s
 *@return: 到达目标点返回1，否则返回0
 */
void TraceMove(int TraceDate,float TarSpeed)
{
	int turnpwm=0;
	int spdpwml=0,spdpwmr=0;
	int pwml=0,pwmr=0;
	
	turnpwm=ChangeTraceTurn(TraceDate);//转向PID
	
	spdpwml=ChangeSpeedMotorL(Motor2.Uinttime_MotorPulse, TarSpeed);
	spdpwmr=ChangeSpeedMotorR(Motor1.Uinttime_MotorPulse, TarSpeed);

//	printf("Encode_Left:%d Encode_Right:%d\r\n",Encode_Left,Encode_Right);
	
	pwmr=turnpwm+spdpwmr;
	if(pwmr>MAX_MOTOR_PWM)		pwmr=MAX_MOTOR_PWM;//限幅
	else if(pwmr<-MAX_MOTOR_PWM)  pwmr=-MAX_MOTOR_PWM;
	
	pwml=-turnpwm+spdpwml;
	if(pwml>MAX_MOTOR_PWM)		pwml=MAX_MOTOR_PWM;//限幅
	else if(pwml<-MAX_MOTOR_PWM)  pwml=-MAX_MOTOR_PWM;
	
	Set_PWM(pwml, pwmr);
}
