#ifndef __CAR_H
#define __CAR_H

#include "control.h"

#define Green_Set HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);
#define Green_ReSet HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);
#define Red_Set HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET);
#define	Red_ReSet HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET);

#define LEFT 5
#define RIGHT 6

typedef struct _TARFET {
	uint8_t number;
	uint8_t direction;
	
} TARGET_t;

extern TARGET_t target;
extern uint8_t rec;

float Car_Turn(int16_t angle);
void Key_Proc(void);

#endif
