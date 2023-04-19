/*****************************************************/
//Code for ADC

#include "ADC.h"

void Init_ADC(void){
	//Using PC0 for ADC
	GPIOC->MODER |= 3; 		// Set PC0 to Analog mode
	GPIOC->PUPDR &= ~(3); // Set PC0 to no pull up or down
	
	//Using PC3 CH 13
	GPIOC->MODER |= ((1 << 7) | (1 << 6));
	GPIOC->PUPDR &= ~((1 << 7) | (1 << 6));
	
	//Enable ADC1 in RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	
	ADC1->CFGR1 |= 1 << 4;       // Set to 8 bit resolution
	ADC1->CFGR1 |= 1 << 13;      // Set to Continuous conversion mode
	//ADC1->CFGR1 &= ~(3 << 10); // Disable hardware tigger detection
	
	ADC1->CHSELR |= 1 << 10;     //Select PC0 / channel 10
	ADC1->CHSELR |= 1 << 13;
	
	//Set to wait conversion
	ADC1->CFGR1 |= 1 << 14;

	//Calibrate ADC
	//1. Ensure that ADEN = 0 and DMAEN = 0.
	ADC1->CR &= ~(1);			// Set ADEN to off
	ADC1->CFGR1 &= ~(1);	// Set DMAEN to off
	
  //2. Set ADCAL = 1.
  ADC1->CR |= 1 << 31;
	
	//3. Wait until ADCAL = 0.
	while((ADC1->CR & (1 << 31)) == (1 << 31)){}
	ADC1->CR |= 1; //Enable ADC
		
  //4. The calibration factor can be read from bits 6:0 of ADC_DR.
	uint8_t calibrationFactor = (ADC1->DR & 0x3F);
		
	ADC1->CR |= 1 << 2; //Start ADC
}

void Read_ADCs(uint8_t* left, uint8_t* right){
	//Wait for PC0 to finish converting
	while((ADC1->ISR & ADC_ISR_EOC) == 0){} 
		
	//Store the ADC data to the left integer
	*left = ADC1->DR;
	
	//Wait for PC3 to finish converting
	while((ADC1->ISR & ADC_ISR_EOC) == 0){}
		
	//Store the ADC data to the right integer
	*right = ADC1->DR;
}