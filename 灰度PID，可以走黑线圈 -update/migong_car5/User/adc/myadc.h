#ifndef __MYADC_H
#define __MYADC_H

#include "main.h"
#include "gpio.h"

uint16_t Get_Adc(uint8_t Channel);
uint16_t Get_Adc_Average(uint8_t ch, uint8_t times); 

#endif
