#ifndef __PID_H
#define __PID_H

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _PID {
	float Kp;
	float Ki;
	float Kd;
	
	int Motor1_Out;  // 电机1输出
	int Motor2_Out;  // 电机2输出
	
} PID_t;
extern PID_t Location;  // 位置环
extern PID_t Velocity;  // 速度环
extern PID_t Incremental;// 增量式
extern PID_t Position;   // 位置式

void PID_Init(void);
void Straight_Trail_param(void);

float Location_PID_Realize(float Location_Actual_Val, float Location_Target_Val);
float Velocity_PID_Realize(float Velocity_Actual_Val, float Velocity_Target_Val);
float Turn_PID_Realize(float Bias);
float Position_PID_Realize(float reality, float target);
float Incremental_PID_Realize(float reality, float target);


#endif
