#ifndef OUR_USART_H
#define OUR_USART_H

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

void Init_USART(void);
void Transmit_String(char input[]);
void Transmit_USART(char symbol);

#endif