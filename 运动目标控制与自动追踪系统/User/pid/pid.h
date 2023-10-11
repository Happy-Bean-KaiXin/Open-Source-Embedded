#ifndef __PID_H
#define __PID_H

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _PID {
	float Kp;
	float Ki;
	float Kd;
	
	int Servo_TarXout;  // 电机1输出
	int Servo_TarYout;  // 电机2输出
	
} PID_t;

extern PID_t Location;  // 位置环
extern PID_t Velocity;  // 速度环
extern PID_t Incremental;// 增量式

extern PID_t Servo_Position; 



void PID_Init(void);

float Location_PID_Realize(float Location_Actual_Val, float Location_Target_Val);

float Velocity_PID_Realize(float Velocity_Actual_Val, float Velocity_Target_Val);
float Velocity_PID_Realize1(float Velocity_Actual_Val, float Velocity_Target_Val);

float Turn_PID_Realize(float Bias);

// 位置式PID，使用红色激光笔当前位置，移动目标位置控制舵机转动的角度
float Position_PID_RealizeX(float reality, float target);
float Position_PID_RealizeY(float reality, float target);

float Incremental_PID_Realize(float reality, float target);


#endif
