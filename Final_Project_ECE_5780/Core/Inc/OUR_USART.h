#ifndef OUR_USART_H
#define OUR_USART_H

#include <stdio.h>
#include <stdlib.h>
#include "stm32f0xx.h"

void Transmit_USART(char symbol);
void Transmit_String(char input[]);
void Init_USART(void);

#endif