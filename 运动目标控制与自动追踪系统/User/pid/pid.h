#ifndef __PID_H
#define __PID_H

#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

typedef struct _PID {
	float Kp;
	float Ki;
	float Kd;
	
	int Servo_TarXout;  // ���1���
	int Servo_TarYout;  // ���2���
	
} PID_t;

extern PID_t Location;  // λ�û�
extern PID_t Velocity;  // �ٶȻ�
extern PID_t Incremental;// ����ʽ

extern PID_t Servo_Position; 



void PID_Init(void);

float Location_PID_Realize(float Location_Actual_Val, float Location_Target_Val);

float Velocity_PID_Realize(float Velocity_Actual_Val, float Velocity_Target_Val);
float Velocity_PID_Realize1(float Velocity_Actual_Val, float Velocity_Target_Val);

float Turn_PID_Realize(float Bias);

// λ��ʽPID��ʹ�ú�ɫ����ʵ�ǰλ�ã��ƶ�Ŀ��λ�ÿ��ƶ��ת���ĽǶ�
float Position_PID_RealizeX(float reality, float target);
float Position_PID_RealizeY(float reality, float target);

float Incremental_PID_Realize(float reality, float target);


#endif
