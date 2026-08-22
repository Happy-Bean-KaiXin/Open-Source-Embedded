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


// [BUGFIX] Encoder left/right mapping. This sets WHICH wheel each speed-loop regulates;
// it affects BOTH straight driving and turning (NOT just left/right turns).
//   TIM1(htim1) -> Motor1,  TIM3(htim3) -> Motor2        (see encoder.c).
//   Set_PWM(pwml, pwmr): pwml -> TIM2_CH3 (left motor), pwmr -> TIM2_CH4 (right motor)  (see motor.c).
// Default (line below is COMMENTED): LEFT speed-loop reads Motor2, RIGHT reads Motor1.
//   * If the physical LEFT wheel is on Motor1 (TIM1):  the default is WRONG -> UNCOMMENT below.
//   * If the physical LEFT wheel is on Motor2 (TIM3):  the default is correct -> leave commented.
// Decide from the SCHEMATIC, do NOT guess: a wrong mapping cross-couples the two speed loops
// (straight-line drift + high-speed jitter); it does not merely flip the turn direction.
// #define MOTOR_LEFT_IS_MOTOR1   // <- 取消注释的前提：TIM1->Motor1 接的是左轮

#endif
