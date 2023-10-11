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

// �궨��
#define Mode1_SET HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "RecognizeRed", strlen("RecognizeRed"))
#define Mode2_SET HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "RecognizeRetangle", strlen("RecognizeRetangle"))
#define Mode3_SET HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "CorrectRedSpots", strlen("CorrectRedSpots"))

// �������ݷ����ַ�
#define Identify_Upleft    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "upleft", strlen("upleft"))
#define Identify_Upright   HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "upright", strlen("upright"))
#define Identify_Lowright  HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "lowright", strlen("lowright"))
#define Identify_Lowleft   HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "lowleft", strlen("lowleft"))
#define Identify_Midpoint  HAL_UART_Transmit_DMA(&huart2, (uint8_t *) "midpoint", strlen("midpoint"))

// ��������
extern int RetangleX[4];                    // ���ο�x�������
extern int RetangleY[4];                    // ���п�y�������
extern float anglex, angley;                  // x�ᣬy��ĽǶ�ֵ
extern int Black_Retanx[4];                                      // ��ɫ���ο�����x
extern int Black_Retany[4];                                      // ��ɫ���ο�����y

void print(UART_HandleTypeDef* huart, const char* buf, ...);   // ָ�����ڷ��ͺ���
void OLED_Proc();
void VOFA_Proc();

float calculateSlope(float x1, float y1, float x2, float y2);  // б��
int calculateIntercept(int x1, int y1, float slope);           // �ؾ�
int calculateY(int x, float slope, int intercept);             // ����x��y
int calculateX(float y, float slope, float intercept);         // ����y��x
int myabs(int p);

void PARAM_Init(void);
void Motion_TarCtrl(int* RetangleX, int* RetangleY);
void Motion_TarCtrl_Black(int* Black_Retanx, int* Black_Retany);

// ��־λö�ٶ���
enum __FLAGSTATE {
		Centy_To_Start = 0,
		Start_To_Second,
		Second_To_Thrid,
		Thrid_To_Fourth,
		Fourth_To_End,
};

typedef struct _PARAM {
	// ��־λ��������
	uint8_t Is_Angle_Set;
	uint8_t Is_10ms_YES;
	uint8_t Is_OLED_Face;
	enum __FLAGSTATE FSTATE;
	uint8_t Is_Mode_Set;    // 0 ��λģʽ 1 ����ģʽ 2�������������ģʽ
	uint8_t Is_Uart_Rec;    // 0 ʶ����ģʽ  1 ʶ����ο�ģʽ
	
	uint8_t TEST;
	uint8_t Cnt;
	
//	uint8_t Is_Centy_To_Start;
//	uint8_t Is_Start_To_Second;
//	uint8_t Is_Second_To_Thrid;
//	uint8_t Is_Thrid_To_Fourth;
//	uint8_t Is_Fourth_To_End;
	
	// �������� 
	float   Anglex;    // ��ʼ����ջ���ĽǶ�x, y
	float   Angley;
	int     X_AXIS;    // ����x
	int			Y_AXIS;    // ����y
	int     x_actual;  // x�����ʵ��ֵ
	int     y_actual;  // y�����ʵ��ֵ
	int     x_centry;  // x�����ĵ�����ֵ
	int     y_centry;  // y�����ĵ�����ֵ
	float   Slope;     // б��
	int     Intercpet; // �ؾ�
	int     dx;        // ����x�Ĳ�ֵ
	int     dy;        // ����y�Ĳ�ֵ
	
} PARAM_t;

extern PARAM_t Flag;

// ��ɫ����״̬����
enum Laser_STATE {
	Box_Square_State = 0,
	
	/////////////////////////////
	Any_Rectang_Box_State,
	Cross_State,
	
	/***** ״̬1 *******/
	Centry_X_Start_State,
	Centry_Y_Start_State,
	
	/***** ״̬2 *******/
	Start_X_Second_State,
	Start_Y_Second_State,
	
	/***** ״̬3 *******/
	Second_X_Thrid_State,
	Second_Y_Thrid_State,
	
	/***** ״̬4 *******/
	Thrid_X_Fourth_State,
	Thrid_Y_Fourth_State,
	
	/***** ״̬5 *******/
	Fourth_X_End_State,
	Fourth_Y_End_State,
};

// �˶�Ŀ�����ϵͳ�ṹ��
typedef struct _LASER {
	enum Laser_STATE Laser_State; 
	
} LASER_t;

extern LASER_t RED_LASER;

#endif
