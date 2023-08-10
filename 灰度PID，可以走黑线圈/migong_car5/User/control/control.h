#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "bsp_oled.h"
#include "track.h"
#include "myadc.h"

//��������ת��
#define MOTOR_SPEED_MAX  270  // ��λrpm

//PID�ļ�������
#define PID_COMPUTATION_PERIOD  10//��λ��ms

void OLED_Proc(void);
void TraceMove(int TraceDate,float TarSpeed);

#endif
