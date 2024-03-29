/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t buttonPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GPIO_Init(void);
void EXTI0_IRQHandler(void);
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
  GPIO_Init();
  uint32_t current_state;
  	  uint32_t current_state1;

  	uint32_t blink_delay = 1000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

////PIRMA DALIS
//	  GPIOD->BSRR = GPIO_BSRR_BR_13;
//	  GPIOD->BSRR = GPIO_BSRR_BR_14;
//	  HAL_Delay(1000);
//	  GPIOD->BSRR = GPIO_BSRR_BS_13;
//	  HAL_Delay(1000);
//	  GPIOD->BSRR = GPIO_BSRR_BR_13;
//	  GPIOD->BSRR = GPIO_BSRR_BS_14;
//	  HAL_Delay(1000);
//	  GPIOD->BSRR = GPIO_BSRR_BS_13;
//	  HAL_Delay(1000);
//Antra dalis

//	  current_state = GPIOD->IDR & GPIO_IDR_IDR_14;
//	  current_state1 = GPIOD->IDR & GPIO_IDR_IDR_13;
//	  HAL_Delay(1000);
//	  GPIOD->BSRR = current_state ? GPIO_BSRR_BR_14 : GPIO_BSRR_BS_14;
//	  current_state = GPIOD->IDR & GPIO_IDR_IDR_14;
//	  	  current_state1 = GPIOD->IDR & GPIO_IDR_IDR_13;
//	  HAL_Delay(1000);
//	  GPIOD->BSRR = current_state ? GPIO_BSRR_BR_14 : GPIO_BSRR_BS_14;
//	  GPIOD->BSRR = current_state1 ? GPIO_BSRR_BR_13 : GPIO_BSRR_BS_13;
//	  current_state = GPIOD->IDR & GPIO_IDR_IDR_14;
//	  	 current_state1 = GPIOD->IDR & GPIO_IDR_IDR_13;
//	HAL_Delay(1000);
//	 GPIOD->BSRR = current_state ? GPIO_BSRR_BR_14 : GPIO_BSRR_BS_14;
//	 current_state = GPIOD->IDR & GPIO_IDR_IDR_14;
//	 	  current_state1 = GPIOD->IDR & GPIO_IDR_IDR_13;
//	HAL_Delay(1000);
//	GPIOD->BSRR = current_state1 ? GPIO_BSRR_BR_13 : GPIO_BSRR_BS_13;
//	GPIOD->BSRR = current_state ? GPIO_BSRR_BR_14 : GPIO_BSRR_BS_14;
//	current_state = GPIOD->IDR & GPIO_IDR_IDR_14;
//		  current_state1 = GPIOD->IDR & GPIO_IDR_IDR_13;

//PAPILDOMA
//.1
//	  if (buttonPressed)
//	       {
//	           // Toggle the state of the green LED (PD12)
//		  GPIOD->BSRR = GPIO_BSRR_BS_12;
//
//	         //  buttonPressed = 0;  // Reset the button press flag
//	       }
//2.2
	  if (buttonPressed)
	  	       {
	  //	           // Toggle the state of the green LED (PD12)
	  blink_delay+=500;
	  	           buttonPressed = 0;  // Reset the button press flag
	  	       }
	  		  GPIOD->BSRR = GPIO_BSRR_BS_13;
	  		HAL_Delay(blink_delay);
	  		GPIOD->BSRR = GPIO_BSRR_BR_13;
	  		HAL_Delay(blink_delay);




    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void GPIO_Init(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;;

	GPIOD->MODER |= GPIO_MODER_MODER12_0;
	GPIOD->MODER |= GPIO_MODER_MODER13_0;
	GPIOD->MODER |= GPIO_MODER_MODER14_0;
	GPIOD->MODER |= GPIO_MODER_MODER15_0;

	 GPIOA->MODER &= ~GPIO_MODER_MODER0;   // Clear mode bits for PA0
	    GPIOA->PUPDR |= GPIO_PUPDR_PUPDR0_0;  // Enable pull-up for PA0

	    // Configure EXTI for the user button (PA0)
	    EXTI->IMR |= EXTI_IMR_MR0;      // Enable interrupt on EXTI line 0 (connected to PA0)
	    EXTI->RTSR |= EXTI_RTSR_TR0;    // Trigger on rising edge (button press)

	    // Configure NVIC for EXTI0_IRQn
	    NVIC_SetPriority(EXTI0_IRQn, 0);  // Set priority (you can adjust this as needed)
	    NVIC_EnableIRQ(EXTI0_IRQn);       // Enable EXTI0_IRQn interrupt
}
void EXTI0_IRQHandler(void)
{
    if (EXTI->PR & EXTI_PR_PR0)  // Check if EXTI line 0 triggered the interrupt
    {
        EXTI->PR |= EXTI_PR_PR0;  // Clear the pending bit

        buttonPressed = 1;  // Set the flag to indicate button press
    }
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
