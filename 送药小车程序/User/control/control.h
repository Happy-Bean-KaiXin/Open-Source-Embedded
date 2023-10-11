#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "bsp_oled.h"
#include "track.h"
#include "myadc.h"
#include "stdio.h"
#include "usart.h"
#include "main.h"
#include "stdbool.h"
#include "car.h"

#define pi 3.1415926

//��������ת��
#define MOTOR_SPEED_MAX  220  // ��λrpm

//PID�ļ�������
#define PID_COMPUTATION_PERIOD  10//��λ��ms

typedef struct _PARAM {
	
	uint8_t Is_Location_Speed;
	uint8_t OLED_FACE;
	uint8_t Is_Track;         // ѭ�����ܱ�־λ
	uint8_t Is_Medicine;      // ���ҩƷ��־λ  1 Ϊ��ҩƷ   0Ϊ��ҩƷ
	uint8_t Is_DisArrive;     // �������Ƿ񵽴��־λ
	uint8_t Is_Select_Left;
	uint8_t Is_Select_Right;
	// ����
	uint8_t Is_NumOne;
	uint8_t Is_NumTwo;
	uint8_t Is_DIs_MID_FAR;   // ���ڽ��յ���Զ�����֣����˱�־λ��Ϊ1������ȴ�״̬�����ж�
	uint8_t Is_NumThree;
	uint8_t Is_NumFour;
	uint8_t Is_NumFive;
	uint8_t Is_NumSix;
	uint8_t Is_NumSeven;
	
	// ȷ��������ȷ���
	uint8_t Is_Sure_NumThree;    // �±�0 ��ʾ��ʼʶ�𵽵�Ŀ�����֣� �±�1��ʾ����ʶ����Ŀ������ �� �±�2��ʾ����ʶ����Ŀ������   �����������
	uint8_t Is_Sure_NumFour;
	uint8_t Is_Sure_NumFive;
	uint8_t Is_Sure_NumSix;
	uint8_t Is_Sure_NumSeven;
	uint8_t Is_Num_Recongnize;     // ����ʶ��
	
	//
	uint8_t Is_Left_Log;     // ��¼��ת��Ĵ���
	uint8_t Is_Right_Log;
	uint8_t Is_Left_Turn;
	uint8_t Is_Right_Turn;
	uint8_t Is_Car_Stop;
	uint8_t Is_Mid_Dis;       // �����Զ�˵����֣����˱�־λ��Ϊ1�����ں�������·�̵ĵ��жϣ�������ȴ�״̬
	
} PARAM_t;

extern PARAM_t Flag;

enum CAR_STATE {
//	Init_PID_State = 0,
//	Init_Thre_State,
		Test_Medicine_State = 0,
		Follow_Line_State,
		Cross_State,
		Select_Left_State,
		Select_Right_State,
		Turn_Around_State,     // �Ƿ����ת��180
		Stop_State,
		Select_MID_Left_State,
		Select_MID_Right_State,
		Select_FAR_Left_State,
		Select_FAR_Right_State,
		Turn_Follow_State,
		Keep_Fllow_Straight_State,
};

typedef struct _CAR {
	
	enum CAR_STATE Car_State;
	float Car_Dis;
	long MotorPluse[2];
} CAR_t;

extern CAR_t CAR;

void Param_Init(void);
void OLED_Proc(void);
void Vofa_Proc(void);
void TraceMove(int TraceDate,float TarSpeed);
void TraceMoveWithDis(int TraceDate,float TarDis);
void MoveDis(int NowEncodeLALL,int NowEncodeRALL,float TarDisL,float TarDisR);
void Car_State_Machine(void);

#endif
