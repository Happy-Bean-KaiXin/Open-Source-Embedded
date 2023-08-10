#include "pid.h"

PID_t Location;   // λ�û�
PID_t Velocity;   // �ٶȻ�

PID_t Incremental;// ����ʽ
PID_t Position;   // λ��ʽ

PID_t Turn_PID;       // ת��

void PID_Init(void) {
	
	// λ�û�PID����
	Location.Kp = 0.00f;
	Location.Ki = 0.00f;
	Location.Kd = 0.0f;
	
	
	// �ٶȻ�PID����
	Velocity.Kp = -0.0f;
	Velocity.Ki = -0.0f;
	Velocity.Kd = -0.0f;	
}

// ֱ��ѭ������
void Straight_Trail_param(void) {
	
	Position.Kp = -30.00f;
	Position.Ki = -2.600f;
	Position.Kd = 0.0f;
	
	Turn_PID.Kp = -1.2f;
	Turn_PID.Kd = -0.0f;
}

/**
* @brief:λ�û�PID
* @param:
* 			[in]Location_Actual_Val: ����ʵ��ֵ
* 			[in]Location_Target_Val: ����Ŀ��ֵ
* @return:λ�û�PID���ֵ
*/
float Location_PID_Realize(float Location_Actual_Val, float Location_Target_Val) {
	static float Location_Error = 0.0;//λ�û���ƫ��
	static float Location_ErrorLast = 0.0;//λ�û�����һ��ƫ��
	static float Location_Integral = 0.0;//λ�û��Ļ���ֵ
	static float Location_Out = 0.0;//���PWM
  //1.����ƫ��
	Location_Error=Location_Target_Val-Location_Actual_Val;
	
	//2.�ۼ�ƫ��
	Location_Integral+=Location_Error;
	
	//3.�����޷�
	if(Location_Integral>5000)Location_Integral=5000;
	if(Location_Integral<-5000)Location_Integral=-5000;
	
	//3.PID�㷨ʵ��
	Location_Out=(Location.Kp*Location_Error)
								+(Location.Ki*Location_Integral)
								+(Location.Kd*(Location_Error-Location_ErrorLast));
	
	//4.ƫ��Ĵ���
	Location_ErrorLast=Location_Error;
	
	//5.����λ�û�����õ������ֵ
	return Location_Out;  

}

/**
* @brief:�ٶȻ�PID
* @param:
* 			[in]Velocity_Actual_Val: ����ʵ��ֵ
* 			[in]Velocity_Target_Val: ����Ŀ��ֵ
* @return:�ٶȻ�PID���ֵ
*/
float Velocity_PID_Realize(float Velocity_Actual_Val, float Velocity_Target_Val) { 
	static float Velocity_Error = 0.0;//�ٶȻ���ƫ��
	static float Velocity_ErrorLast = 0.0;//�ٶȻ�����һ��ƫ��
	static float Velocity_Integral = 0.0;//�ٶȻ�
	static float Velocity_Out = 0.0;
  //1.����ƫ��
	Velocity_Error=Velocity_Target_Val-Velocity_Actual_Val;
	
	Velocity_Integral+=Velocity_Error;
		//3.�����޷�
	if(Velocity_Integral>5000)Velocity_Integral=5000;
	if(Velocity_Integral<-5000)Velocity_Integral=-5000;
	//3.PID�㷨ʵ��
	Velocity_Out=(Velocity.Kp*Velocity_Error)
								+(Velocity.Ki*Velocity_Integral)
								+(Velocity.Kd*(Velocity_Error-Velocity_ErrorLast));
	
	//4.ƫ��Ĵ���

	
	Velocity_ErrorLast = Velocity_Error;
	
	
	//7.�����ٶȻ�����õ������ֵ
	return Velocity_Out;  
}

/**
* @brief:λ��ʽPID
* @param:
* 			[in]reality: ����ʵ��ֵ
* 			[in]target: ����Ŀ��ֵ
* @return:λ��ʽPID���ֵ
*/
float Position_PID_Realize(float reality, float target) { 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target-reality;                            /* ����ƫ�� */
    Integral_bias+=Bias;	                        /* ƫ���ۻ� */
    
    if(Integral_bias> 5000) Integral_bias = 5000;   /* �����޷� */
    if(Integral_bias<-5000) Integral_bias =-5000;
    
    Pwm = (Position.Kp*Bias)                        /* �������� */
         +(Position.Ki*Integral_bias)               /* ���ֻ��� */
         +(Position.Kd*(Bias-Last_Bias));           /* ΢�ֻ��� */
    
    Last_Bias=Bias;                                 /* �����ϴ�ƫ�� */
    return Pwm;                                     /* ������ */
}


/**
* @brief:����ʽPID
* @param:
* 			[in]reality: ����ʵ��ֵ
* 			[in]target: ����Ŀ��ֵ
* @return:����ʽPID���ֵ
*/
float Incremental_PID_Realize(float reality, float target) {
	
	 static float Bias, Pwm, Last_bias = 0.0, Prev_bias=0.0;
    
	 Bias = target-reality;                                       /* ����ƫ�� */
    
	 Pwm += (Incremental.Kp*(Bias-Last_bias))                     /* �������� */
           +(Incremental.Ki*Bias)                               /* ���ֻ��� */
           +(Incremental.Kd*(Bias - 2*Last_bias + Prev_bias));  /* ΢�ֻ��� */ 
    
   Prev_bias = Last_bias;                                       /* �������ϴ�ƫ�� */
	 Last_bias = Bias;	                                          /* ������һ��ƫ�� */
    
	 return Pwm;                                                  /* ������ */
}

/**
* @brief:ת��PID
* @param:
* 			[in]Bias: �������
* 
* @return:ת��PID���ֵ
*/
float Turn_PID_Realize(float Bias) {
		static float Turn, Last_Bias = 0.0;
	
	  Turn = Turn_PID.Kp * Bias + Turn_PID.Kd * (Last_Bias-Bias);
		Last_Bias = Bias;
	  return Turn;
}
