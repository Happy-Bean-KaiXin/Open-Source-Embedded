#include "servo.h"

float Init_CompareX;//���ռ�ձȳ�ʼֵ
float Init_CompareY;//���ռ�ձȳ�ʼֵ

float Angle_Flag_X=0.0;
float Angle_Flag_Y=0.0;

/*
 * �����ʼ�� ��ʼ�Ƕ�
 *
 */
void Servo_Angle_Init(void)
{
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);		// ʹ��PWM���  A15  x��
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);		// ʹ��PWM���  B3   y��
}

/* X������ʼ��΢��
 * Angle:�����Ƕ�
 * Direction:��������
 */
void Servo_X_Angle_Set(float Angle)
{
    Init_CompareX =50+((((-1)*Angle+95)*(1/90.0))/20*TIM2_ARR);//90�Ȳ�����50ms����
    Angle_Flag_X=Angle;//��ǰ�Ƕ�
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Init_CompareX);
}
/* Y������ʼ��΢��
 * Angle:�����Ƕ�
 * Direction:��������
 */
void Servo_Y_Angle_Set(float Angle)
{
    Init_CompareY =50+((((-1)*Angle+95)*(1/90.0))/20*TIM2_ARR);//90�Ȳ�����50ms����
    Angle_Flag_Y=Angle;//��ǰ�Ƕ�
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Init_CompareY);  // ���ö��ת��PWM
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ�����position ��ǰ���ת��λ�õĶ�ʱ���Ƚ�ֵ
         target    ���������ת�ĽǶ�
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
int Servo_Position_PIDY(int position, int target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias = target-position;                                  // ����ƫ��
	 Integral_bias += Bias;	                                    // ���ƫ��Ļ���
	 LIMIT(Integral_bias, -5000, 5000);                       // �����޷�
	 Pwm = (Servo_Position.Kp/100)*Bias+(Servo_Position.Ki/100)*Integral_bias+(Servo_Position.Kd/100)*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                          // ������һ��ƫ�� 
	 return Pwm;                                              // �������
}

int Servo_Position_PIDX(int position, int target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias = target-position;                                  // ����ƫ��
	 Integral_bias += Bias;	                                    // ���ƫ��Ļ���
	 LIMIT(Integral_bias, -5000, 5000);                       // �����޷�
	 Pwm = (Servo_Position.Kp/100)*Bias+(Servo_Position.Ki/100)*Integral_bias+(Servo_Position.Kd/100)*(Bias-Last_Bias);       //λ��ʽPID������
	 Last_Bias=Bias;                                          // ������һ��ƫ�� 
	 return Pwm;                                              // �������
}

// ����Ƕȷ��أ����ض�ʱ���Ƚ�ֵ
int Servo_Angle_Compare(float Angle) {
   return  50+(((-Angle+95)*(1/90.0))/20*TIM2_ARR);//90�Ȳ�����50ms����
}

// ���붨ʱ���ıȽ�ֵ�������������Ƕ�
float Servo_Compare_Angle(uint32_t Compare) {
    return -(((Compare-50.0)*20*90.0)/TIM2_ARR - 95);
}

void Set_Pwm_ServoX(int motox) {
	int Position;
	Position += motox;
	
	if(Position>1250) Position=1250;
	if(Position<250) Position=250;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Position);  // ���ö��ת��PWM
}

void Set_Pwm_ServoY(int motoY) {
	int Position;
	Position += motoY;
	
	if(Position>1250)Position=1250;
	if(Position<250)Position=250;
	
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Position);  // ���ö��ת��PWM
}
/*********************************************END OF FILE*****************************************/






