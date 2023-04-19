/*****************************************************/
//Code for USART

#include "OUR_USART.h"

//Transmitting a Character
void Transmit_USART(char symbol){
	// Check to see if Transmit Data register is empty
	while(1){
		if((USART3->ISR & (1 << 7)) == (1 << 7)){
			break;
		}
	}
	// Write character into Transmit Data Register
	USART3->TDR = symbol;
	// Hardware will clear the status bit for us
	// for the status register for transmit register
}

//Transmitting a String
void Transmit_String(char input[]){
	int i = 0;
	while(input[i] != NULL){
		Transmit_USART(input[i]);
		i++;
	}
}

void Init_USART(void){
	/**************************************************/
	// Exercise 4.1
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->MODER |= 1 << 21; // Alternate function for PB10
	GPIOB->MODER |= 1 << 23; // Alternate function for PB11
	GPIOB->AFR[1] |= 1 << 10; //Set the AFR to AF4 to select USART3 TX
	GPIOB->AFR[1] |= 1 << 14; //Set the AFR to AF4 to select USART3 RX
	/****************************************************/
	
	/*****************************************************/
	//Interrupt Based Recption
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
	/*****************************************************/
	
	/****************************************************/
	//Initializing the USART
	// Enable USART3 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	// Set BRR to get our desire baud rate 8MHz / Target Baud Rate
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600;//115200;
	
	// This is apart of section 4.9.5 Interrupt Based Rection
	USART3->CR1 |= 1 << 5;
	// Ends here
	
	USART3->CR1 |= 1 << 3; // Enables Transmitter
	USART3->CR1 |= 1 << 2; // Enables Receiver
	USART3->CR1 |= 1;
	/*****************************************************/
	
	return;
}