#ifndef __SERVO_H
#define __SERVO_H

#include "contorl.h"

#define SERVO_X_INIT_ANGLE		(0)//���X�����Ƕ�
#define SERVO_Y_INIT_ANGLE		(0)//���Y�����Ƕ�

/*TIM2*/
#define TIM2_ARR							(2000-1)
#define TIM2_PSC							(720-1)

void Servo_Angle_Init(void);
void Servo_X_Angle_Set(float Angle);
void Servo_Y_Angle_Set(float Angle);

int Servo_Angle_Compare(float Angle);
float Servo_Compare_Angle(uint32_t Compare);

int Servo_Position_PIDY(int position, int target);
int Servo_Position_PIDX(int position, int target); 

void Set_Pwm_ServoX(int motox);
void Set_Pwm_ServoY(int motoY);

// ��������
extern float Init_CompareX;//���ռ�ձȳ�ʼֵ
extern float Init_CompareY;//���ռ�ձȳ�ʼֵ

extern float Angle_Flag_X;
extern float Angle_Flag_Y;

#endif //SERVO_SERVO_H
