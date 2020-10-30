/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  NOT_BLINKING = 0U,
  BLINKING,
  TRANSITION_BLINK_TO_NOT_BLINKING,
  TRANSITION_TO_NOT_BLINKING
}Button_state_machine_t;

typedef enum
{
  BUTTON_PRESSED = 0U,
  BUTTON_RELEASED
}Button_status_t;

typedef struct
{
  uint16_t active_cnt;
  uint16_t deactive_cnt;
  const uint16_t hysteresis_cnt;
  Button_status_t current_btn_state_t;
}debounce_button_status_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BL_DEBUG_MSG_EN

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static void printmsg(char *format,...);

void init_debounce_struct_t(debounce_button_status_t *);

void init_debounce_struct_t(debounce_button_status_t *struct_t)
{
	struct_t->active_cnt = 0;
	struct_t->current_btn_state_t = BUTTON_RELEASED;
	struct_t->deactive_cnt = 0;
}

void debounc_btn(debounce_button_status_t *,const GPIO_PinState);

void debounc_btn(debounce_button_status_t *btn_status_t,const GPIO_PinState GPIO_return_t)
{
	uint32_t overflow_cntr = 0;
	if(GPIO_return_t == GPIO_PIN_RESET)
	{
		overflow_cntr = btn_status_t->active_cnt +1;
		if(overflow_cntr>65535)
		{
			btn_status_t->active_cnt = 65535;
		}
		else
		{
			btn_status_t->active_cnt = (uint16_t)overflow_cntr;
		}
		if(btn_status_t->active_cnt >= btn_status_t->hysteresis_cnt)
		{
			btn_status_t->current_btn_state_t = BUTTON_PRESSED;
		}
		else
		{
			btn_status_t->current_btn_state_t = BUTTON_RELEASED;
		}


	}
	else
	{
		btn_status_t->active_cnt = 0;
		btn_status_t->current_btn_state_t = BUTTON_RELEASED;
		overflow_cntr = btn_status_t->active_cnt +1;
		if(overflow_cntr>65535)
		{
			btn_status_t->deactive_cnt = 65535;
		}
		else
		{
			btn_status_t->deactive_cnt = (uint16_t)overflow_cntr;
		}
	}

}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define D_UART &huart2
#define BL_RX_LEN 200
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static GPIO_PinState gpio_status_t = GPIO_PIN_RESET;
static debounce_button_status_t user_btn_status_t = {.hysteresis_cnt = 50};
static Button_state_machine_t button_state_t = NOT_BLINKING;
static HAL_StatusTypeDef uart_status_t;
char somedata[] = "Hello from Bootloader\r\n";
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  init_debounce_struct_t(&user_btn_status_t);
  gpio_status_t = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port,USER_BTN_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*
	  gpio_status_t = HAL_GPIO_ReadPin(USER_BTN_GPIO_Port,USER_BTN_Pin);
	  debounc_btn(&user_btn_status_t,gpio_status_t);
	  switch(button_state_t)
	  {
	  	  case NOT_BLINKING:
	  		if(user_btn_status_t.current_btn_state_t == BUTTON_PRESSED)
	  		{
	  			button_state_t = BLINKING;
	  		}
	  		else
	  		{
	  			//do nothing
	  			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	  		}
	  		  break;

	  	  case BLINKING:
	  		if(user_btn_status_t.current_btn_state_t == BUTTON_RELEASED)
	  		{
	  			button_state_t = TRANSITION_BLINK_TO_NOT_BLINKING;
	  		}
	  		else
	  		{
	  			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  			HAL_Delay(1);
	  		}
	  		break;
	  	  case TRANSITION_BLINK_TO_NOT_BLINKING:
	  		if(user_btn_status_t.current_btn_state_t == BUTTON_PRESSED)
	  		{
	  			button_state_t = TRANSITION_TO_NOT_BLINKING;
	  		}
	  		else
	  		{
	  			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  			HAL_Delay(1);
	  		}
	  		  break;
	  	  case TRANSITION_TO_NOT_BLINKING:
	  		if(user_btn_status_t.current_btn_state_t == BUTTON_RELEASED)
	  		{
	  			button_state_t = NOT_BLINKING;
	  		}
	  		else
	  		{
	  			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  			HAL_Delay(1);
	  		}
	  		  break;
	  		default:
	  		  break;

	  }
	  HAL_Delay(50);
	  //
	   *
	   */
	  //uart_status_t = HAL_UART_Transmit(&huart2,(uint8_t*)somedata,sizeof(somedata),HAL_MAX_DELAY);
	  printmsg(somedata);
	  uint32_t current_tick = HAL_GetTick();
	  printmsg("current_tick = %d\r\n",current_tick);
	  while(HAL_GetTick() <= (current_tick+500));

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**************Implementation of Boot-loader Command Handle functions *********/

void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
	char str[80];
	va_list args;
	va_start(args,format);
	vsprintf(str,format,args);
	HAL_UART_Transmit(D_UART,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);
	va_end(args);
#endif
}

/*Helper function to handle BL_GET_VER command */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(char *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
