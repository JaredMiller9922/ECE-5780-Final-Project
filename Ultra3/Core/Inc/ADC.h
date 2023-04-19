#ifndef OUR_ADC
#define OUR_ADC

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

void Init_ADC(void);
void Read_ADCs(uint8_t* left, uint8_t* right);

#endif