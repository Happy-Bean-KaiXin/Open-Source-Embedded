#include "myadc.h"
#include "adc.h"



//  只用通道0 1 4 5 8
/****完全轮询模式****/
uint16_t Get_Adc(uint8_t Channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = Channel;                                         /* 通道 */
	sConfig.Rank = ADC_REGULAR_RANK_1;                              
	sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;                  /* 采样时间 */
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)             
	{
		Error_Handler();
	}
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	return (uint16_t)HAL_ADC_GetValue(&hadc1);
}

uint16_t Get_Adc_Average(uint8_t ch, uint8_t times) 
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t = 0; t < times; t++)
	{
		temp_val+=Get_Adc(ch);
		HAL_Delay(3);
	}
	return temp_val/times;
}



