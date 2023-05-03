/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
	* 
	* This is the main.c file for our ECE-5780 Final Project. The group consists
	* of Jared Miller, Hyrum Bailey, and Misael Nava
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motor.h"
#include "OUR_USART.h"
#include "ADC.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint32_t Is_First_Captured = 0;
uint32_t Distance = 0;

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA

#define MoveForward (1)
#define StopMoving  (2)
#define TurnLeft		(3)
#define TurnRight		(4)
#define Turn180R		(5)
#define Turn180L		(6)
#define Turn45R			(7)
#define Turn45L			(8)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile char input = 'x';
TIM_HandleTypeDef htim1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */
	
	uint8_t leftADC = 0;
	uint8_t rightADC = 0;
	
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	// LED_init();
  LED_init();
  MX_GPIO_Init();
  MX_TIM1_Init();
	motor_init();
	Init_USART();
	Init_ADC();
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

	/******************************************************************/
	//State Machine setup code
	//Flags for Ultrasonic and Flex Resistors
	uint8_t objectOnLeft = 0;		//Left Flex Resistor
	uint8_t objectOnRight = 0;  //Right Flex Resistor
	uint8_t objectIsClose = 0;  //For Ultrasonic
	
	uint16_t state = StopMoving;
	
  /* USER CODE END 2 */
	char leftWarning[] = "Object is on our left\n\r";
	char rightWarning[] = "Object is on our right\n\r";
	char frontWarning[] = "Object is in front\n\r";
	
	//char test[] = "test";
	char owo[] = "LEFT";
	char umu[] = "RIGHT";
	char uzu[] = "Swithced left";
	char uxu[] = "swithced right";
	char buffer[5];
	
	int counter = 0;
	/* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {			
		//Here check ADCs and Ultrasonic to set flags
		HCSR04_Read();
		HCSR04_Read();
		HCSR04_Read();
		//itoa(Distance, buffer, 10);
		sprintf(buffer, "%d", Distance);//counter);
		Transmit_String(buffer);
		Transmit_USART('\r');
		Transmit_USART('\n');
		HAL_Delay(10);
		if(Distance <= 10){//Double check if 10cm is good enough
			objectIsClose = 1;
		}
		else{
			objectIsClose = 0;
		}
		
		//Read twice to get better readings, first reading might be not good
		//Read_ADCs(&leftADC, &rightADC);
		//Read_ADCs(&leftADC, &rightADC);
		Read_ADCs(&rightADC, &leftADC);
		Read_ADCs(&rightADC, &leftADC);
		if(leftADC > 195){
			objectOnLeft = 1;
		}
		else{
			objectOnLeft = 0;
		}
		if(rightADC > 195){
			objectOnRight = 1;
		}
		else{
			objectOnRight = 0;
		}
		
		//Figure out which state we should be in
		if(state == MoveForward && objectIsClose && objectOnLeft && objectOnRight){
			state = StopMoving;
			Transmit_String(frontWarning);
			Transmit_String(leftWarning);
			Transmit_String(rightWarning);
		}
		else if(state == MoveForward && objectOnLeft && objectOnRight){
			state = StopMoving;
			Transmit_String(leftWarning);
			Transmit_String(rightWarning);
		}
		else if(state == MoveForward && objectIsClose){
			//Set State to stop moving
			state = StopMoving;
			
			//Send USART message saying stop moving because object is too
			//close in front
			Transmit_String(frontWarning);
		}
		else if(state == MoveForward && objectOnLeft){
			//Set State to stop Moving
			state = StopMoving;
			
			//Send USART message saying stop moving becasue object
			//is close to the left of the rover
			Transmit_String(leftWarning);
		}
		else if(state == MoveForward && objectOnRight){
			//Set State to stop Moving
			state = StopMoving;
			
			//Send USART message saying stop moving becasue object
			//is close to the right of the rover
			Transmit_String(rightWarning);
		}
		else if(state == StopMoving){
			//Choose next state based off USART input
			switch(input){
				case 'f':
					state = MoveForward;
					input = 'x'; //Set input back to default
					break;
				case 'l':
					state = TurnLeft;
					input = 'x';
					break;
				case 'r':
					state = TurnRight;
					input = 'x';
					break;
				case 'b':
					state = Turn180R;
					input = 'x';
					break;
				case 'v':
					state = Turn180L;
					input = 'x';
					break;
				case 'q':
					state = Turn45L;
					input = 'x';
					break;
				case 'w':
					state = Turn45R;
					input = 'x';
					break;
				default:
					state = StopMoving;
			}
		}
		else if(state == TurnLeft){
			//Finished turning left, need to stay in stop state
			state = StopMoving;
		}
		else if(state == TurnRight){
			//Finished turning right, need to stay in stop state
			state = StopMoving;
		}
		else if(state == Turn180R){
			//Finished turning 180, need to stay in stop state
			state = StopMoving;
		}
		else if(state == Turn180L){
			state = StopMoving;
		}
		else if(state == Turn45R){
			state = StopMoving;
		}
		else if(state == Turn45L){
			state = StopMoving;
		}
		else{
			//Stay in the same state if there is no neeed to change
			state = state;
		}
		
		//Switch based of state and do the work that needs to be done in that state
		switch(state){
			case MoveForward:
				//Set motors speed to what we want
					//pwm_setDutyCycle_LMTR(73);
					//pwm_setDutyCycle_RMTR(74);
					if(counter <= 14){
						pwm_setDutyCycle_LMTR(74);
						pwm_setDutyCycle_RMTR(64);
					}
					else{
						pwm_setDutyCycle_LMTR(64);
						pwm_setDutyCycle_RMTR(73);
						if(counter == 18){counter = 0;}
					}
					counter++;
					break;
			case StopMoving:
				//Turn off motors
				pwm_setDutyCycle_LMTR(0);
				pwm_setDutyCycle_RMTR(0);
				break;
			case TurnLeft:
				//Turn right motor for a certain amount of time then stop it
				rotate90Left();
				counter = 0;
				break;
			case TurnRight:
				//Turn left motor for a certain amount of time then stop it
				rotate90Right();
				counter = 0;
				break;
			case Turn180R:
				rotate180Right();
				counter = 0;
				break;
			case Turn180L:
				rotate180Left();
				counter = 0;
				break;
			case Turn45R:
				rotate45Right();
				counter = 0;
				break;
			case Turn45L:
				rotate45Left();
				counter = 0;
				break;
			}
			
		HAL_Delay(100);
		
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}
/* USER CODE BEGIN 4 */
void USART3_4_IRQHandler(void){
		input = USART3->RDR;
}
/*
void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}
*/
void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while(__HAL_TIM_GET_COUNTER(&htim1)< time);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (65535 - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
