/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.h
  * @brief          : Header for pid.c file.
  ******************************************************************************
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

// 限幅函数，将x的值限制在min 与 max之间
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

extern float Kp, Kd;

typedef struct {
	float Kp, Ki, Kd; 
	
	int Out;
	
}PIDTypedef;

typedef struct _CascadePID
{
	PIDTypedef inner;    // 内环速度环PID
	PIDTypedef outer;    // 外环角度环PID
	float output;
}CascadePID;

extern PIDTypedef VerTical;
extern PIDTypedef VeloCity;

/*! -------------------------------------------------------------------------- */
/*! Public functions list */
int Vertical(float Med, float Angle, float gyro_y);
int Velocity(int Moto_Left, int Moto_Right);
int Turn(int gyro_z);
void print(UART_HandleTypeDef* huart, const char* buf, ...);

void Key_Proc(void);

void PID_Init(void);
void PID_Clear(PIDTypedef *pid);
void PID_SetMaxOutput(PIDTypedef *pid);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
