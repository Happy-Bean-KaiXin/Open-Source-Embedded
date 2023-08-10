#include "control.h"
#include "stdio.h"


#define pi 3.1415926
float WheelOneCircleDis=WheelDiameter*pi;//����С����������һȦ�������

// ��ʱ���жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM4){
		// ��ȡ��ǰ��λʱ������
		Motor1.Uinttime_MotorPulse = -GetMotorPulse(1);  
		Motor2.Uinttime_MotorPulse = -GetMotorPulse(3);
		
		// �����ۼ�������
		Motor1.Sigma_MotorPulse += Motor1.Uinttime_MotorPulse;
		Motor2.Sigma_MotorPulse += Motor2.Uinttime_MotorPulse;
		
		// ����С����ǰ��ʻ����   ��������/ÿһȦ�������� * �ܳ�  == ����
		Motor1.Distance = Motor1.Sigma_MotorPulse/ACircleEncoder * pi * WheelDiameter;
		Motor2.Distance = Motor2.Sigma_MotorPulse/ACircleEncoder * pi * WheelDiameter;
		
		// ����ȡ��������ת��Ϊrpm
//		Motor1.speed = ((float)Motor1.Uinttime_MotorPulse)/ACircleEncoder * 6000.0f;   // ��10msת��Ϊ����Ҫ����6000  ����ACircleEncoderΪ�궨�� 1560��������һȦ�������������
//		Motor2.speed = ((float)Motor2.Uinttime_MotorPulse)/ACircleEncoder * 6000.0f;   // ��10msת��Ϊ����Ҫ����6000
		
		// ����ȡ������ת��Ϊm/s
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
 *@brief:����ѭ��������pid����С��ת��ʹС�����ں����м�
 * @param:
 *        [in]int TraceDate: ѭ�������������ֵ
 * @return: ���ص��ڵ���ٶȵ�ת��pwm
 */
int ChangeTraceTurn(int TraceDate)
{
	int pwm=0;
	int bias;
	bias = TraceDate;
	pwm = Turn_PID_Realize( bias);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:����pid������ߵ����Ŀ���ٶ�
 * @param:
 *        [in]int EncodeSpdL: ��ǰ��������������ֵ
 *        [in]float TarSpdL:��ߵ��Ŀ���ٶ�,����ٶ�Խ1.19m/s
 * @return: ������ߵ��������pwmռ�ձ�
 */
int ChangeSpeedMotorL(int NowEncodeSpdL,float TarSpdL)
{
	int pwm=0;
	int TarEncodeSpdL;
	TarEncodeSpdL=(int)((TarSpdL*ACircleEncoder)/(WheelOneCircleDis*100));//����Ŀ���ٶ����Ŀ��������ٶ�

	pwm=Position_PID_Realize(NowEncodeSpdL, TarEncodeSpdL);
	
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	
	return pwm;
}

/*@brief:����pid�����ұߵ����Ŀ���ٶ�
 * @param:
 *        [in]int NowEncodeSpdR: ��ǰ�ҵ������������ֵ
 *        [in]float TarSpdR:�ұߵ��Ŀ���ٶ�,�����޷�����ٶ�ԼΪ0.7m/s
 * @return: �����ұߵ��������pwmռ�ձ�
 */
int ChangeSpeedMotorR(int NowEncodeSpdR,float TarSpdR)
{
	int pwm=0;
	int TarEncodeSpdR;
	TarEncodeSpdR=(int)((TarSpdR*ACircleEncoder)/(WheelOneCircleDis*100));//����Ŀ���ٶ����Ŀ��������ٶ�
	
	pwm=Position_PID_Realize(NowEncodeSpdR, TarEncodeSpdR);
	if(pwm>MAX_MOTOR_PWM)		pwm=MAX_MOTOR_PWM;//�޷�
	else if(pwm<-MAX_MOTOR_PWM)  pwm=-MAX_MOTOR_PWM;
	return pwm;
}

/**
 *@brief: ��С������ѭ��������
 *@param:
 *        [in]TraceDate: ѭ�������������ֵ
 *        [in]TarSpeed:ѭ����Ŀ���ٶ�,�ٶȴ�С0-0.7m/s
 *@return: ����Ŀ��㷵��1�����򷵻�0
 */
void TraceMove(int TraceDate,float TarSpeed)
{
	int turnpwm=0;
	int spdpwml=0,spdpwmr=0;
	int pwml=0,pwmr=0;
	
	turnpwm=ChangeTraceTurn(TraceDate);//ת��PID
	
	spdpwml=ChangeSpeedMotorL(Motor2.Uinttime_MotorPulse, TarSpeed);
	spdpwmr=ChangeSpeedMotorR(Motor1.Uinttime_MotorPulse, TarSpeed);

//	printf("Encode_Left:%d Encode_Right:%d\r\n",Encode_Left,Encode_Right);
	
	pwmr=turnpwm+spdpwmr;
	if(pwmr>MAX_MOTOR_PWM)		pwmr=MAX_MOTOR_PWM;//�޷�
	else if(pwmr<-MAX_MOTOR_PWM)  pwmr=-MAX_MOTOR_PWM;
	
	pwml=-turnpwm+spdpwml;
	if(pwml>MAX_MOTOR_PWM)		pwml=MAX_MOTOR_PWM;//�޷�
	else if(pwml<-MAX_MOTOR_PWM)  pwml=-MAX_MOTOR_PWM;
	
	Set_PWM(pwml, pwmr);
}
