#include "encoder.h"

void Encoder_Init(void) {
	
	// ����������
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1); // ����������A 
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2); // ����������B;	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // ����������A 
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // ����������B;	
}

/**
 * @function: int GetMotorPulse(TIM_HandleTypeDef* htim)
 * @description: ʹ��STM32������ģʽ����ȡ����������������ֵ
 * @param {*} 
 * @return {*}
 */
int GetMotorPulse(int timx) {
	int encoder_counter;   //STM32������ģʽ��ȡ����������
	switch(timx) {
		case 1: 
			encoder_counter = (short) __HAL_TIM_GET_COUNTER(&htim1); // ��ȡ��������ֵ
			__HAL_TIM_SET_COUNTER(&htim1,0);   // ����������
			break;
		case 3: 
			encoder_counter = (short) __HAL_TIM_GET_COUNTER(&htim3);
			__HAL_TIM_SET_COUNTER(&htim3,0);			
			break;
		default: encoder_counter = 0;
	}  
  return  encoder_counter;               
}

/**
 * @function: ����ת����Ӧ������������
 * @description: ʹ��STM32������ģʽ����ȡ����������������ֵ
 * @param 
				[in]num: ����С����Ȧ��
 * @return  
 */
long  Nlaps_Encoder_Cnt(float num) {
	return num * ACircleEncoder;         /* 4��Ƶ */
}

/**************************************************************************
��    ��: ����ת�ٶ�Ӧ������������
��    ��: rpm��ת�٣�ppr����������ratio�����ٱȣ�cnt_time������ʱ��(ms)
�� �� ֵ: ���������  תrpm�����������
**************************************************************************/
long Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time)
{
    return (rpm*ratio*ppr*4)/(60*1000/cnt_time);            /* 4��Ƶ */       
}

