#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"

/***********************��������������**************************/
#define ACircleLeftEncode 1560.0f	//һȦ �ֱܷ��ʣ�13*4��*���ٱȣ�30��
#define ACircleRightEncode 1560.0f	//һȦ1560
#define ACircleEncoder ACircleLeftEncode 

#define RR 30u                // ���ٱ�

void Encoder_Init(void);
int  GetMotorPulse(int timx);
long Nlaps_Encoder_Cnt(float num);
long Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time);

#endif
