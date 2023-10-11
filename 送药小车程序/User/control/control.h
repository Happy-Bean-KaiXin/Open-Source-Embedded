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

//电机的最大转速
#define MOTOR_SPEED_MAX  220  // 单位rpm

//PID的计算周期
#define PID_COMPUTATION_PERIOD  10//单位是ms

typedef struct _PARAM {
	
	uint8_t Is_Location_Speed;
	uint8_t OLED_FACE;
	uint8_t Is_Track;         // 循迹功能标志位
	uint8_t Is_Medicine;      // 检测药品标志位  1 为有药品   0为无药品
	uint8_t Is_DisArrive;     // 检测距离是否到达标志位
	uint8_t Is_Select_Left;
	uint8_t Is_Select_Right;
	// 数字
	uint8_t Is_NumOne;
	uint8_t Is_NumTwo;
	uint8_t Is_DIs_MID_FAR;   // 串口接收到中远段数字，将此标志位置为1，放入等待状态进行判断
	uint8_t Is_NumThree;
	uint8_t Is_NumFour;
	uint8_t Is_NumFive;
	uint8_t Is_NumSix;
	uint8_t Is_NumSeven;
	
	// 确认数字正确与否
	uint8_t Is_Sure_NumThree;    // 下标0 表示开始识别到的目标数字， 下标1表示向左识别到了目标数字 ， 下标2表示向右识别到了目标数字   这个方法再议
	uint8_t Is_Sure_NumFour;
	uint8_t Is_Sure_NumFive;
	uint8_t Is_Sure_NumSix;
	uint8_t Is_Sure_NumSeven;
	uint8_t Is_Num_Recongnize;     // 数字识别
	
	//
	uint8_t Is_Left_Log;     // 记录左转弯的次数
	uint8_t Is_Right_Log;
	uint8_t Is_Left_Turn;
	uint8_t Is_Right_Turn;
	uint8_t Is_Car_Stop;
	uint8_t Is_Mid_Dis;       // 如果是远端的数字，将此标志位置为1，便于后续返回路程的的判断，不放入等待状态
	
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
		Turn_Around_State,     // 是否进行转向180
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
