/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[20];
uint8_t TxBuffer[200];

enum State {INIT, LED_CONTROL, BUTTON_STATUS};
enum State processState = INIT;
uint8_t text[200];

typedef struct ButtonStatus
{
	int8_t Command;
	int8_t CurrentStatus;
	int8_t PreviousStatus;
}Button;
Button button1 = {0};

typedef struct LEDStatus
{
	float    Frequency;
	int8_t   OnOffStatus;
}LED;
LED led = {50.0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTPollingMethod();
void UARTDMAConfig();
void DummyTask();
void ButtonUpdate();

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  sprintf((char*)text, "-----Program Start-----\r\n");
  HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
  //print(text);
  UARTDMAConfig();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //UARTPollingMethod();
	  static uint32_t timestampBt = 0;
	  	if(HAL_GetTick() >= timestampBt)
		{
			timestampBt = HAL_GetTick() + 100;
			DummyTask();
			ButtonUpdate();
		}

	  //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
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
  huart2.Init.BaudRate = 57600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void UARTPollingMethod()
//{
//	//Read UART 10 chars within 10 s
//	HAL_StatusTypeDef HAL_status = HAL_UART_Receive(&huart2, RxBuffer, 10, 10000);
//
//	//Read 10 chars complete in 10 s
//	if(HAL_status == HAL_OK)
//	{
//		//Add str stop symbol \0 to the end of str
//		RxBuffer[10] = '\0';
//
//		//Return received chars
//		sprintf((char*)TxBuffer, "Recieved : %s\r\n", RxBuffer);
//		HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), 10);
//	}
//	//Timeout : print only rrecieved chars
//	else if (HAL_status == HAL_TIMEOUT)
//	{
//		//Add str stop symbol \0 to the end of str
//		uint32_t lastCharInd = huart2.RxXferSize - huart2.RxXferCount;
//		RxBuffer[lastCharInd] = '\0';
//
//		//Return recieved chars
//		sprintf((char*)TxBuffer, "Recieved Timeout: %s\r\n", RxBuffer);
//		HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), 10);
//	}
//}

void UARTDMAConfig()
{
//	//Start UART in Interrupt mode
	//HAL_UART_Receive_IT(&huart2, RxBuffer, 10);

	//Start UART in DMA mode
	HAL_UART_Receive_DMA(&huart2, RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		//Add str stop symbol \0 to the end of str
		RxBuffer[1] = '\0';

		//Return received chars
		sprintf((char*)TxBuffer, "Recieved Command: %s\r\n", RxBuffer);
		HAL_UART_Transmit(&huart2, TxBuffer, strlen((char*)TxBuffer), 100);

		//Recall Recieved
		//HAL_UART_Receive_IT(&huart2, RxBuffer, 10);
		switch(processState)
			{
			case INIT:
				if(RxBuffer[0] == '0')
				{
					processState = LED_CONTROL;
					sprintf((char*)text, "MODE : LED_CONTROL\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
				}
				else if(RxBuffer[0] == '1')
				{
					processState = BUTTON_STATUS;
					sprintf((char*)text, "MODE : BUTTON_STATUS\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					button1.Command = 1;
				}
				else
				{
					sprintf((char*)text, "Not a command.\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					processState = INIT;
				}
			break;

			case LED_CONTROL:

				if(RxBuffer[0] == 'x')
				{
					processState = INIT;
					sprintf((char*)text, "BACK\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
				}
				else if(RxBuffer[0] == 'a')
				{
					led.Frequency += 1;
					processState = LED_CONTROL;
					sprintf((char*)text, "LED Frequency +1 Hz\r\nCurrent Frequency = %d\r\n", led.Frequency);
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
				}
				else if(RxBuffer[0] == 's')
				{
					if(led.Frequency > 0)
					{
						led.Frequency -= 1;
						sprintf((char*)text, "LED Frequency -1 Hz\r\nCurrent Frequency = %d\r\n", led.Frequency);
						HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					}
					else
					{
						sprintf((char*)text, "Frequency can not less than zero.\r\n");
						HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					}
					processState = LED_CONTROL;
				}
				else if(RxBuffer[0] == 'd')
				{

					if(led.OnOffStatus == 1)
					{
						led.OnOffStatus = 0;
						sprintf((char*)text, "LED : OFF\r\n");
						HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					}
					else if(led.OnOffStatus == 0)
					{
						led.OnOffStatus = 1;
						sprintf((char*)text, "LED : ON\r\n");
						HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					}


					processState = LED_CONTROL;
				}
				else
				{
					sprintf((char*)text, "Not a command.\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					processState = LED_CONTROL;
				}


			break;

			case BUTTON_STATUS:

				if(RxBuffer[0] == 'x')
				{
					button1.Command = 0;
					sprintf((char*)text, "BACK\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					processState = INIT;
				}
				else
				{
					sprintf((char*)text, "Not a command.\r\n");
					HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
					processState = BUTTON_STATUS;
				}

			break;
			}
	}
}

void DummyTask()
{

	if(button1.Command == 1 && button1.CurrentStatus == 1)
	{

		sprintf((char*)text, "Button Status : Unpress \r\n");
		HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
	}
	else if(button1.Command == 1 && button1.CurrentStatus == 0)
	{
		sprintf((char*)text, "Button Status : Press \r\n");
		HAL_UART_Transmit_DMA(&huart2, text, strlen((char*)text));
	}

	static uint32_t timestampLED = 0;
	if(led.OnOffStatus == 1)
	{
		//Blink LED 5 Hz
		if(HAL_GetTick() >= timestampLED)
		{
			timestampLED = HAL_GetTick() + (1.0/led.Frequency)*1000;
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
	}
	else if(led.OnOffStatus == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		timestampLED = 0;
	}
}

void ButtonUpdate()
{
	button1.CurrentStatus =  HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
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
