/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char TxDataBuffer[32] = { 0 }; //output text
char RxDataBuffer[32] = { 0 }; //input char
int16_t Inputchar = 0;
uint8_t State = 000;
uint64_t Timestamp_LED = 0;
int Freq_LED = 1;
uint16_t Timedelay_LED = 0;
uint8_t Switch[2] = {0};
int LED_mode = 0;
char Menu_Start[] = "0: LED Control\r\n1: Button status\r\n";
char Menu_LED[] = "a: Speed up\r\ns: Speed Down\r\nd: On/Off\r\nx: Back\r\n";
char Menu_Button[] = "x: Back\r\nButton status:\r\n";
char Buttonpress[] = "Button Press\r\n";
char Buttonunpress[] = "Button Unpress\r\n";
char WrongInput[] = "Wrong input... Try again\r\n";
char LED_Freq[20] = "";
char LED_On[] = "LED On\r\n";
char LED_Off[] = "LED Off\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int16_t UARTRecieveIT();
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
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_UART_Receive_IT(&huart2, (uint8_t*)RxDataBuffer, 32);
		Inputchar = UARTRecieveIT();
		Switch[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		if(Inputchar > 0) // != Null
		{
			sprintf(TxDataBuffer,"Input: %c\r\n",Inputchar);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 10);
		}
		switch(State)
		{
			case 000: //Start state
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Start, strlen(Menu_Start), 10);
				State = 001;
				break;
			case 001: //Choose mode
				if(Inputchar == '0')
				{
					State = 010;
				}
				else if(Inputchar == '1')
				{
					State = 011;
				}
				else if(Inputchar > 0)
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)WrongInput, strlen(WrongInput), 10);
					State = 000;
				}
				break;
			case 010: //Led state
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_LED, strlen(Menu_LED), 10);
				State = 100;
				break;
			case 100: //Led control
				if(Inputchar == 'a')
				{
					Freq_LED = Freq_LED + 1;
					sprintf(LED_Freq,"Frequency: %d\r\n",Freq_LED);
					HAL_UART_Transmit(&huart2, (uint8_t*)LED_Freq, strlen(LED_Freq), 10);
					State = 010;
				}
				else if(Inputchar == 's')
				{
					Freq_LED = Freq_LED - 1;
					if(Freq_LED < 0)
					{
						Freq_LED = 0;
					}
					sprintf(LED_Freq,"Frequency: %d\r\n",Freq_LED);
					HAL_UART_Transmit(&huart2, (uint8_t*)LED_Freq, strlen(LED_Freq), 10);
					State = 010;
				}
				else if(Inputchar == 'd')
				{
					if(LED_mode == 1) //LED off
					{
						LED_mode = 0;
						HAL_UART_Transmit(&huart2, (uint8_t*)LED_Off, strlen(LED_Off), 10);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
					}
					else if(LED_mode == 0)
					{
						LED_mode = 1;
						HAL_UART_Transmit(&huart2, (uint8_t*)LED_On, strlen(LED_On), 10);
					}
					State = 010;
				}
				else if(Inputchar == 'x')
				{
					State = 000;
				}
				else if(Inputchar > 0)
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)WrongInput, strlen(WrongInput), 10);
					State = 010;
				}
				break;
			case 011: // Button menu
				HAL_UART_Transmit(&huart2, (uint8_t*)Menu_Button, strlen(Menu_Button), 10);
				State = 101;
				break;
			case 101: //Button status
				if(Switch[0] == 0 && Switch[1] == 1) //press
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)Buttonpress, strlen(Buttonpress), 10);
				}
				else if(Switch[0] == 1 && Switch[1] == 0) //unpress
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)Buttonunpress, strlen(Buttonunpress), 10);
				}
				if(Inputchar == 'x')
				{
					State = 000;
				}
				else if(Inputchar > 0)
				{
					HAL_UART_Transmit(&huart2, (uint8_t*)WrongInput, strlen(WrongInput), 10);
					State = 011;
				}
				break;
		}

		Switch[1] = Switch[0];

		if(LED_mode == 1)
		{
			Timedelay_LED = 500/Freq_LED;
			if(HAL_GetTick() - Timestamp_LED > Timedelay_LED)
			{
				Timestamp_LED = HAL_GetTick();
				HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			}
		}

    /* USER CODE END WHILE */

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	sprintf(TxDataBuffer, "Received:[%s]\r\n", RxDataBuffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
