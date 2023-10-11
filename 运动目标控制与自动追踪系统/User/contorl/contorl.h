#ifndef __CONTORL_H
#define __CONTORL_H

#include "main.h"
#include "tim.h"
#include <stdint.h>
#include "uart.h"
#include "servo.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"
#include "usart.h"
#include "bsp_oled.h"
#include "key.h"

// 宏定义
#define Mode1_SET HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "RecognizeRed", strlen("RecognizeRed"))
#define Mode2_SET HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "RecognizeRetangle", strlen("RecognizeRetangle"))
#define Mode3_SET HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "CorrectRedSpots", strlen("CorrectRedSpots"))

// 矫正数据发送字符
#define Identify_Upleft    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "upleft", strlen("upleft"))
#define Identify_Upright   HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "upright", strlen("upright"))
#define Identify_Lowright  HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "lowright", strlen("lowright"))
#define Identify_Lowleft   HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "lowleft", strlen("lowleft"))
#define Identify_Midpoint  HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "midpoint", strlen("midpoint"))

// 变量定义
extern int RetangleX[4];                    // 矩形框x轴的坐标
extern int RetangleY[4];                    // 举行框y轴的坐标
extern float anglex, angley;                  // x轴，y轴的角度值
extern int Black_Retanx[4];                                      // 黑色矩形框坐标x
extern int Black_Retany[4];                                      // 黑色矩形框坐标y

void print(UART_HandleTypeDef* huart, const char* buf, ...);   // 指定串口发送函数
void OLED_Proc();
void VOFA_Proc();

float calculateSlope(float x1, float y1, float x2, float y2);  // 斜率
int calculateIntercept(int x1, int y1, float slope);           // 截距
int calculateY(int x, float slope, int intercept);             // 根据x求y
int calculateX(float y, float slope, float intercept);         // 根据y求x
int myabs(int p);

void PARAM_Init(void);
void Motion_TarCtrl(int* RetangleX, int* RetangleY);
void Motion_TarCtrl_Black(int* Black_Retanx, int* Black_Retany);

// 标志位枚举定义
enum __FLAGSTATE {
		Centy_To_Start = 0,
		Start_To_Second,
		Second_To_Thrid,
		Thrid_To_Fourth,
		Fourth_To_End,
};

typedef struct _PARAM {
	// 标志位变量定义
	uint8_t Is_Angle_Set;
	uint8_t Is_10ms_YES;
	uint8_t Is_OLED_Face;
	enum __FLAGSTATE FSTATE;
	uint8_t Is_Mode_Set;    // 0 复位模式 1 方框模式 2矩形与任意矩形模式
	uint8_t Is_Uart_Rec;    // 0 识别红点模式  1 识别矩形框模式
	
	uint8_t TEST;
	uint8_t Cnt;
	
//	uint8_t Is_Centy_To_Start;
//	uint8_t Is_Start_To_Second;
//	uint8_t Is_Second_To_Thrid;
//	uint8_t Is_Thrid_To_Fourth;
//	uint8_t Is_Fourth_To_End;
	
	// 变量定义 
	float   Anglex;    // 初始舵机闭环后的角度x, y
	float   Angley;
	int     X_AXIS;    // 坐标x
	int			Y_AXIS;    // 坐标y
	int     x_actual;  // x坐标的实际值
	int     y_actual;  // y坐标的实际值
	int     x_centry;  // x在中心的坐标值
	int     y_centry;  // y在中心的坐标值
	float   Slope;     // 斜率
	int     Intercpet; // 截距
	int     dx;        // 坐标x的差值
	int     dy;        // 坐标y的差值
	
} PARAM_t;

extern PARAM_t Flag;

// 红色激光状态定义
enum Laser_STATE {
	Box_Square_State = 0,
	
	/////////////////////////////
	Any_Rectang_Box_State,
	Cross_State,
	
	/***** 状态1 *******/
	Centry_X_Start_State,
	Centry_Y_Start_State,
	
	/***** 状态2 *******/
	Start_X_Second_State,
	Start_Y_Second_State,
	
	/***** 状态3 *******/
	Second_X_Thrid_State,
	Second_Y_Thrid_State,
	
	/***** 状态4 *******/
	Thrid_X_Fourth_State,
	Thrid_Y_Fourth_State,
	
	/***** 状态5 *******/
	Fourth_X_End_State,
	Fourth_Y_End_State,
};

// 运动目标控制系统结构体
typedef struct _LASER {
	enum Laser_STATE Laser_State; 
	
} LASER_t;

extern LASER_t RED_LASER;

#endif
