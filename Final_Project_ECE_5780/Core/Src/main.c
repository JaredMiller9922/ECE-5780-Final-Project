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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MoveForward (1)
#define StopMoving  (2)
#define TurnLeft    (3)
#define TurnRight   (4)
#define Turn180     (5)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

	// LED_init();
	motor_init();
	pwm_setDutyCycle_DR1(50);
	pwm_setDutyCycle_DR2(100);
	
  /*************************************************************************/
	//State Machine setup code
	// Flags for Ultrasonic and Flex Resistors
	uint8_t objectOnSide = 0;   // For flex resistor
	uint8_t objectIsClose = 0;  // For Ultrasonic
	
	// Character input from USART
	char input = 'x';
	
	// Start in initial state
	uint16_t state = MoveForward;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//GPIOC->ODR ^= GPIO_ODR_9; // Toggle green LED
		//HAL_Delay(128); // Delay 1/8 a second

		/**********************************************************************/
		// State machine code
		// Check UltraSonic
			// Send distance to USART
			// Set Ultrasonic flag if needed
		
		// Check Flags and choose state accordingly
			// Flex resistors flags
			// Ultrasonic Flag
		if(state == MoveForward && (objectIsClose || objectOnSide)){
			state = StopMoving;
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
					state = Turn180;
					input = 'x';
					break;
				default:
					state = StopMoving;
			}
		}
		else if(state == TurnLeft){
			state = StopMoving;
		}
		else if(state == TurnRight){
			state = StopMoving;
		}
		else if(state == Turn180){
			state = StopMoving;
		}
		else{
			state = state;
		}
	
		//Switch case for current state
			// Do the work of each state in here
		if(state == MoveForward){
			//Set motors speed to what we want
		}
		else if(state == StopMoving){
			//Set motorspeed to zero
			//maybe turn off polarity
		}
		else if(state == TurnLeft){
			//Turn on only the right motor for a certain amount of time
		}
		else if(state == TurnRight){
			//Turn on only the left motor for a certain amount of time
		}
		else if(state == Turn180){
			//Turn on only one motor for a certain amount of time
		}
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

/* USER CODE BEGIN 4 */

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
