/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"

void LED_init()
{
	// Enable Peripheral Clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Initialize all LED's
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;
	// Set all pins to output
	GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);
	// Set all pins to push-pull output type
	GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR6_0 | GPIO_OSPEEDR_OSPEEDR6_1) |
											(GPIO_OSPEEDR_OSPEEDR7_0 | GPIO_OSPEEDR_OSPEEDR7_1) |
											(GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
											(GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));
	// Set all pins to low speed
	GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR6_0 | GPIO_PUPDR_PUPDR6_1) |
										(GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR7_1) |
										(GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
										(GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));
	// Set all pins to no pull-up / down
	GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7 |GPIO_ODR_8 | GPIO_ODR_9); 
}

// Sets up the entire motor drive system
void motor_init(void) {
    pwm_init();
}

// Sets up the PWM and direction signals to drive the H-Bridge for Motor_Driver_1 PA4: TIM14 PA5, PA6: GPIO output
void pwm_init(void) {

	// Enable Peripheral Clock for GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	// Enable Peripheral Clock for GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// ---------------------------- MTR 1 Pins ------------------------------ //	
	// Set up a PA0, PA1 as GPIO output pins for motor direction control MT2
  GPIOA->MODER |= ((1 << 0) | (1 << 2));
  //Initialize one direction pin to high, the other low
  GPIOA->ODR |= (1 << 0);
  GPIOA->ODR &= ~(1 << 1);

	// ---------------------------- MTR 2 Pins ------------------------------ //
	// Set up a PA4, PA5 as GPIO output pins for motor direction control MTR1
  GPIOA->MODER |= ((1 << 10) | (1 << 8));
  //Initialize one direction pin to high, the other low
  GPIOA->ODR |= (1 << 4);
  GPIOA->ODR &= ~(1 << 5);
	
	// --------------------------- Standby Pin --------------------------- //
	// Set up PB2 as GPIO output pins for standy pin
	GPIOB->MODER |= (1 << 4);
	// Initialize pin to high
	GPIOB->ODR |= (1 << 2);	
	
	// ------------------------------- TIM3 Config ----------------------- //
	// Enable TIM3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Setup timer period
	TIM3->PSC = 7;
	TIM3->ARR = 1250;
	
	// Configure TIM3_CH1 and TIM3_CH2 to PWM mode
	TIM3->CCMR1 &= ~((1 << 0) | (1 << 1)); // Configure CC1S to output
	TIM3->CCMR1 &= ~((1 << 8) | (1 << 9)); // Configure CC2S to output
	
	// Configure OC1M to PWM mode 1
	TIM3->CCMR1 |= (1 << 6) | (1 << 5); 
	TIM3->CCMR1 &= ~(1 << 4); 
	// Configure OC2M to PWM mode 1
	TIM3->CCMR1 |= (1 << 14) | (1 << 13); 
	TIM3->CCMR1 &= ~(1 << 12);
	
	// Enable the output compare preload for both channels
	TIM3->CCMR1 |= (1 << 3); // CH1
	TIM3->CCMR1 |= (1 << 11); // CH2
	
	// Set the output enable bits for channels 1 & 2
	TIM3->CCER |= (1 << 0); // CH1
	TIM3->CCER |= (1 << 4); // CH2
	
	// Start both duty cycles at 0
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;

	// Configure PC6 and PC7 to alternate function and set their alternate function to 0
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	// Set PC6 To AFM
	GPIOC->MODER |= 1 << 13;
	GPIOC->MODER &= ~(1 << 12);
	// Set PC7 To AFM
	GPIOC->MODER |= 1 << 15;
	GPIOC->MODER &= ~(1 << 14);
	// Set the Alternate Function to 0 for PC6 and PC7
	GPIOC->AFR[0] &= ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7);
	
	// Enable TIM3
	TIM3->CR1 |= TIM_CR1_CEN;
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_RMTR(uint8_t duty) {
    if(duty <= 100) {
        TIM3->CCR2 = ((uint32_t)duty*TIM3->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle_LMTR(uint8_t duty) {
    if(duty <= 100) {
        TIM3->CCR1 = ((uint32_t)duty*TIM3->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

// Make the rover drive backwards
void reverse(){
	// Reverse the polarity of both motors
	// ----------------- MTR 1 ------------- // 
	GPIOA->ODR |= (1 << 1);
  GPIOA->ODR &= ~(1 << 0);
	
	// ----------------- MTR 2 ------------- // 
	GPIOA->ODR |= (1 << 5);
  GPIOA->ODR &= ~(1 << 4);
}

// Make the rover drive forwards
void forward(){
	// Reverse the polarity of both motors
	// ----------------- MTR 1 ------------- // 
	GPIOA->ODR |= (1 << 0);
  GPIOA->ODR &= ~(1 << 1);
	
	// ----------------- MTR 2 ------------- // 
	GPIOA->ODR |= (1 << 4);
  GPIOA->ODR &= ~(1 << 5);
}

void rotate90Left(void){
	pwm_setDutyCycle_LMTR(0);
	pwm_setDutyCycle_RMTR(55);
	HAL_Delay(1250);
	pwm_setDutyCycle_LMTR(0);
	pwm_setDutyCycle_RMTR(0);
}

void rotate90Right(void){
	pwm_setDutyCycle_LMTR(55);
	pwm_setDutyCycle_RMTR(0);
	HAL_Delay(1800);
	pwm_setDutyCycle_LMTR(0);
	pwm_setDutyCycle_RMTR(0);
}