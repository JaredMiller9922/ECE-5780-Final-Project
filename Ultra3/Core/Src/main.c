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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile char input = 'o';
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
	
	uint8_t left = 0;
	uint8_t right = 0;
	
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

	
  /* USER CODE END 2 */
	char test[] = "test";
	char owo[] = "LEFT";
	char umu[] = "RIGHT";
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
        /* USER CODE END WHILE */
		HCSR04_Read();
		if(Distance > 2 && Distance <= 5){
			GPIOC->ODR |= GPIO_ODR_6;
			GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9);
		}
		else if(Distance > 5 && Distance <= 12){
			GPIOC->ODR |= GPIO_ODR_6 | GPIO_ODR_9;
			GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_7);
		}
		else if(Distance > 12 && Distance <= 20){
			GPIOC->ODR |= GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_9;
			GPIOC->ODR &= ~(GPIO_ODR_8);
		}
		else if(Distance > 20 && Distance <= 30){
			GPIOC->ODR |= GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9;
		}
		else
			GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9);
		
		HAL_Delay(200);
		// GPIOC->ODR ^= GPIO_ODR_9; // Toggle green LED
		
		
		Transmit_String(test);
		Transmit_USART(input);
		
		Read_ADCs(&left, &right);
		Read_ADCs(&left, &right);
		if(left > 185){
			Transmit_String(owo);
		}
		if(right > 185){
			Transmit_String(umu);
		}
		
		HAL_Delay(1000);

			
		//Move forwards
		pwm_setDutyCycle_LMTR(50);
		pwm_setDutyCycle_RMTR(50);
	
		HAL_Delay(2000);
		
		Transmit_USART(input);
		
		// Stop
		pwm_setDutyCycle_LMTR(0);
		pwm_setDutyCycle_RMTR(0);
	
		HAL_Delay(2000);
	
		Transmit_USART(input);
	
		// Move backwards 
		reverse();
		pwm_setDutyCycle_LMTR(53);
		pwm_setDutyCycle_RMTR(50);
	
		HAL_Delay(2000);
	
		Transmit_USART(input);
	
		// Stop
		pwm_setDutyCycle_LMTR(0);
		pwm_setDutyCycle_RMTR(0);
	
		// Move forwards
		forward();
	
		// Rotate Right
		HAL_Delay(2000);
		
		Transmit_USART(input);
		
		rotate90Right();
	
		HAL_Delay(2000);
	
		Transmit_USART(input);
	
		// Rotate Left
		rotate90Left();	
		
		Transmit_String(test);
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
