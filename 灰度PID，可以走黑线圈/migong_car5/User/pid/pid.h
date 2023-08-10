#ifndef __PID_H
#define __PID_H

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _PID {
	float Kp;
	float Ki;
	float Kd;
	
	int Motor1_Out;  // ���1���
	int Motor2_Out;  // ���2���
	
} PID_t;
extern PID_t Location;  // λ�û�
extern PID_t Velocity;  // �ٶȻ�
extern PID_t Incremental;// ����ʽ
extern PID_t Position;   // λ��ʽ

void PID_Init(void);
void Straight_Trail_param(void);

float Location_PID_Realize(float Location_Actual_Val, float Location_Target_Val);
float Velocity_PID_Realize(float Velocity_Actual_Val, float Velocity_Target_Val);
float Turn_PID_Realize(float Bias);
float Position_PID_Realize(float reality, float target);
float Incremental_PID_Realize(float reality, float target);


#endif
