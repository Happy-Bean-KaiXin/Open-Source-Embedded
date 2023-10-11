#include "pid.h"

PID_t Location;   // 位置环
PID_t Velocity;   // 速度环

PID_t Incremental;// 增量式

PID_t Positionx;   // x位置式
PID_t Positiony;   // y位置式

PID_t Servo_Position;   // 舵机位置式PID

PID_t Turn_PID;   // 转向环PID


void PID_Init(void) {
	Servo_Position.Kp = 50.0;   // 加上上一次舵机目标值的PID参数
	Servo_Position.Ki = 0;
	Servo_Position.Kd = 60.0;
	
	// 传入坐标输出角度的PID参数
	Positionx.Kp = 5.0;//7
	Positionx.Ki = 0.28;//0.2
	Positionx.Kd = 2.2;
	
	Positiony.Kp = 5.0;//7
	Positiony.Ki = 0.25;//0.2
	Positiony.Kd = 2.2;
}

/**
* @brief:位置环PID
* @param:
* 			[in]Location_Actual_Val: 测量实际值
* 			[in]Location_Target_Val: 给定目标值
* @return:位置环PID输出值
*/
float Location_PID_Realize(float Location_Actual_Val,float Location_Target_Val) { 
	static float Location_Error = 0.0;//位置环的偏差
	static float Location_ErrorLast = 0.0;//位置环的上一次偏差
	static float Location_Integral = 0.0;//位置环的积分值
	static float Location_Out = 0.0;//输出PWM
	
  //1.计算偏差
	Location_Error = Location_Target_Val - Location_Actual_Val;

	//2.累计偏差
	Location_Integral+=Location_Error;
	
	//3.积分限幅
	if(Location_Integral>10000)Location_Integral=10000;
	if(Location_Integral<-10000)Location_Integral=-10000;
	
	//3.PID算法实现
	Location_Out=(Location.Kp*Location_Error)
								+(Location.Ki*Location_Integral)
								+(Location.Kd*(Location_Error-Location_ErrorLast));
	
	//4.偏差的传递
	Location_ErrorLast=Location_Error;
	
	//5.返回位置环计算得到的输出值
	return Location_Out;  
}

/**
* @brief:速度环PID
* @param:
* 			[in]Velocity_Actual_Val: 测量实际值
* 			[in]Velocity_Target_Val: 给定目标值
* @return:速度环PID输出值
*/
float Velocity_PID_Realize(float Velocity_Actual_Val, float Velocity_Target_Val) { 
	static float Velocity_Error = 0.0;//速度环的偏差
	static float Velocity_ErrorLast = 0.0;//速度环的上一次偏差
	static float Velocity_Integral = 0.0;//速度环
	static float Velocity_Out = 0.0;
  //1.计算偏差
	Velocity_Error=Velocity_Target_Val-Velocity_Actual_Val;
	
	Velocity_Integral+=Velocity_Error;
		//3.积分限幅
	if(Velocity_Integral>5000)Velocity_Integral=5000;
	if(Velocity_Integral<-5000)Velocity_Integral=-5000;
	//3.PID算法实现
	Velocity_Out=(Velocity.Kp/100*Velocity_Error)
								+(Velocity.Ki/100*Velocity_Integral)
								+(Velocity.Kd*(Velocity_Error-Velocity_ErrorLast));
	
	//4.偏差的传递

	
	Velocity_ErrorLast = Velocity_Error;
	
	
	//7.返回速度环计算得到的输出值
	return Velocity_Out;  
}

/**
* @brief:速度环PID
* @param:
* 			[in]Velocity_Actual_Val: 测量实际值
* 			[in]Velocity_Target_Val: 给定目标值
* @return:速度环PID输出值
*/
float Velocity_PID_Realize1(float Velocity_Actual_Val, float Velocity_Target_Val) { 
	static float Velocity_Error = 0.0;//速度环的偏差
	static float Velocity_ErrorLast = 0.0;//速度环的上一次偏差
	static float Velocity_Integral = 0.0;//速度环
	static float Velocity_Out = 0.0;
  //1.计算偏差
	Velocity_Error=Velocity_Target_Val-Velocity_Actual_Val;
	
	Velocity_Integral+=Velocity_Error;
		//3.积分限幅
	if(Velocity_Integral>5000)Velocity_Integral=5000;
	if(Velocity_Integral<-5000)Velocity_Integral=-5000;
	//3.PID算法实现
	Velocity_Out=(Velocity.Kp*Velocity_Error)
								+(Velocity.Ki*Velocity_Integral)
								+(Velocity.Kd*(Velocity_Error-Velocity_ErrorLast));
	
	//4.偏差的传递

	
	Velocity_ErrorLast = Velocity_Error;
	
	
	//7.返回速度环计算得到的输出值
	return Velocity_Out;  
}

/**
* @brief:位置式PID
* @param:
* 			[in]reality: 测量实际值
* 			[in]target: 给定目标值
* @return:位置式PID输出值
*/
float Position_PID_RealizeX(float reality, float target) { 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target - reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> 5000) Integral_bias = 5000;   /* 积分限幅 */
    if(Integral_bias<-5000) Integral_bias =-5000;
    
    Pwm = ((Positionx.Kp/100)*Bias)                        /* 比例环节 */
         +((Positionx.Ki/100)*Integral_bias)               /* 积分环节 */
         +((Positionx.Kd/100)*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}

/**
* @brief:位置式PID
* @param:
* 			[in]reality: 测量实际值
* 			[in]target: 给定目标值
* @return:位置式PID输出值
*/
float Position_PID_RealizeY(float reality, float target) { 	
    static float Bias,Pwm,Last_Bias,Integral_bias=0;
    
    Bias=target - reality;                            /* 计算偏差 */
    Integral_bias+=Bias;	                        /* 偏差累积 */
    
    if(Integral_bias> 5000) Integral_bias = 5000;   /* 积分限幅 */
    if(Integral_bias<-5000) Integral_bias =-5000;
    
    Pwm = ((Positiony.Kp/100)*Bias)                        /* 比例环节 */
         +((Positiony.Ki/100)*Integral_bias)               /* 积分环节 */
         +((Positiony.Kd/100)*(Bias-Last_Bias));           /* 微分环节 */
    
    Last_Bias=Bias;                                 /* 保存上次偏差 */
    return Pwm;                                     /* 输出结果 */
}


/**
* @brief:增量式PID
* @param:
* 			[in]reality: 测量实际值
* 			[in]target: 给定目标值
* @return:增量式PID输出值
*/
float Incremental_PID_Realize(float reality, float target) {
	
	 static float Bias, Pwm, Last_bias = 0.0, Prev_bias=0.0;
    
	 Bias = target-reality;                                       /* 计算偏差 */
    
	 Pwm += (Incremental.Kp*(Bias-Last_bias))                     /* 比例环节 */
           +(Incremental.Ki*Bias)                               /* 积分环节 */
           +(Incremental.Kd*(Bias - 2*Last_bias + Prev_bias));  /* 微分环节 */ 
    
   Prev_bias = Last_bias;                                       /* 保存上上次偏差 */
	 Last_bias = Bias;	                                          /* 保存上一次偏差 */
    
	 return Pwm;                                                  /* 输出结果 */
}

/**
* @brief:转向环PID
* @param:
* 			[in]Bias: 本次误差
* 
* @return:转向环PID输出值
*/
float Turn_PID_Realize(float Bias) {
		static float Turn, Last_Bias = 0.0;
	
	  Turn = Turn_PID.Kp * Bias + Turn_PID.Kd * (Last_Bias-Bias);
		Last_Bias = Bias;
	  return Turn;
}
