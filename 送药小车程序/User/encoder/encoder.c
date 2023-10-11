#include "encoder.h"

void Encoder_Init(void) {
	
	// 开启编码器
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1); // 开启编码器A 
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2); // 开启编码器B;	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // 开启编码器A 
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // 开启编码器B;	
}

/**
 * @function: int GetMotorPulse(TIM_HandleTypeDef* htim)
 * @description: 使用STM32编码器模式，读取编码器产生的脉冲值
 * @param {*} 
 * @return {*}
 */
int GetMotorPulse(int timx) {
	int encoder_counter;   //STM32编码器模式读取的总脉冲数
	switch(timx) {
		case 1: 
			encoder_counter = (short) __HAL_TIM_GET_COUNTER(&htim1); // 获取编码器数值
			__HAL_TIM_SET_COUNTER(&htim1,0);   // 计数器清零
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
 * @function: 计算转数对应编码器脉冲数
 * @description: 使用STM32编码器模式，读取编码器产生的脉冲值
 * @param 
				[in]num: 输入小车的圈数
 * @return  
 */
long  Nlaps_Encoder_Cnt(float num) {
	return num * ACircleEncoder;         /* 4倍频 */
}

/**************************************************************************
功    能: 计算转速对应编码器脉冲数
输    入: rpm：转速；ppr：码盘数；ratio：减速比；cnt_time：计数时间(ms)
返 回 值: 电机脉冲数  转rpm所需的脉冲数
**************************************************************************/
long Rpm_Encoder_Cnt(float rpm,uint16_t ppr,uint16_t ratio,uint16_t cnt_time)
{
    return (rpm*ratio*ppr*4)/(60*1000/cnt_time);            /* 4倍频 */       
}

