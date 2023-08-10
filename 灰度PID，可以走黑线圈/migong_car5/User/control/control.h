#ifndef __CONTROL_H
#define __CONTROL_H

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "bsp_oled.h"
#include "track.h"
#include "myadc.h"

//电机的最大转速
#define MOTOR_SPEED_MAX  270  // 单位rpm

//PID的计算周期
#define PID_COMPUTATION_PERIOD  10//单位是ms

void OLED_Proc(void);
void TraceMove(int TraceDate,float TarSpeed);

#endif
