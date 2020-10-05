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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFSZ 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t u1RxBuf[BUFSZ] = {0};
uint8_t u2RxBuf[BUFSZ] = {0};
volatile uint16_t u1RxCnt = 0;
volatile uint16_t u1TxCnt = 0;
volatile uint16_t u2RxCnt = 0;
volatile uint16_t u2TxCnt = 0;
volatile uint8_t atret = 0;
volatile uint8_t espInited = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void InitESP8266(){
    HAL_UART_Transmit(&huart1, "AT+CWMODE=3\r\n", 13, 100);
    HAL_Delay(3000);
    HAL_UART_Transmit(&huart1, "AT+CWJAP=\"yaya\",\"yayayaya\"\r\n", 35, 100); // ADD WiFI SSID and password
    HAL_Delay(10000);
    HAL_UART_Transmit(&huart1, "AT+CIPSTART=\"TCP\",\"things.ubidots.com\",9012\r\n", 45, 100);
    HAL_Delay(500);
    HAL_UART_Transmit(&huart1, "ATE0\r\n", 6, 100);
    HAL_Delay(500);
}
void processUart1Received(){
    static uint16_t checkpoint = 0;
    static uint16_t rxCnt = 0;
    if (u1TxCnt != u1RxCnt){
        __disable_irq();
        rxCnt = u1RxCnt;
        __enable_irq();
        // transmit received from UART1 to UART2
        if (u1TxCnt < rxCnt){
            HAL_UART_Transmit(&huart2, &u1RxBuf[u1TxCnt], rxCnt - u1TxCnt, 10);
        }
        else{ // circular case
            HAL_UART_Transmit(&huart2, &u1RxBuf[u1TxCnt], BUFSZ - u1TxCnt, 10);
            HAL_UART_Transmit(&huart2, &u1RxBuf[0], rxCnt, 10);
        }
        // process commands by lines

        static uint8_t teststr[BUFSZ] = {0};
        uint16_t receivedCnt = 0;
        if (checkpoint < rxCnt){
            strncpy(teststr, &u1RxBuf[checkpoint], rxCnt - checkpoint);
            receivedCnt = rxCnt - checkpoint;
        }
        else if(checkpoint > rxCnt){
            strncpy(teststr, &u1RxBuf[checkpoint], BUFSZ - checkpoint);
            strncpy(teststr + BUFSZ - checkpoint, &u1RxBuf[0], rxCnt);
            receivedCnt = BUFSZ - checkpoint + rxCnt;
        }
        for (uint16_t i = 0; i < receivedCnt; i++){
            if (teststr[i] == '\n'){
                char *ipdcmd = strstr(&teststr[i], "+IPD");
                if (ipdcmd != NULL){
                    char *delim = strstr(ipdcmd, "|");
                    if (delim){
                        uint8_t led = *(delim + 1) - '0';
                        if (led == 0){
                            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
                        }
                        else if (led == 1){
                            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
                        }
                    }
                }

            }
        }
        /**/
        // checkpoint = rxCnt;
        u1TxCnt = rxCnt;

    }
}
void processUart2Received()
{
    static uint16_t rxCnt = 0;
    if (u2TxCnt != u2RxCnt){
        __disable_irq();
        rxCnt = u2RxCnt;
        __enable_irq();
        if (u2TxCnt < rxCnt){
            HAL_UART_Transmit(&huart1, &u2RxBuf[u2TxCnt], rxCnt - u2TxCnt, 10);
        }
        else{
            HAL_UART_Transmit(&huart1, &u2RxBuf[u2TxCnt], BUFSZ - u2TxCnt, 10);
            HAL_UART_Transmit(&huart1, &u2RxBuf[0], rxCnt, 10);
        }
        u2TxCnt = rxCnt;

    }
}
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart1, &u1RxBuf[u1RxCnt], 1);
    HAL_UART_Receive_IT(&huart2, &u2RxBuf[u2RxCnt], 1);
    HAL_Delay(1000);
//    InitESP8266();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    uint8_t count = 0;
    while (1)
    {
        processUart1Received();
        processUart2Received();
        HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if(!espInited)
        	continue;
        if(espInited == 1){
        	InitESP8266();
        	espInited = 2;
        }
        else if(count++ > 9){
            static uint8_t val = 0;
            static uint8_t ubidotsbuf[] = "STM32/1|LV|BBFF-08q19mQ1vYLAzQSeKiCxyr7XKTdIgf|123:val|end"; // REPLACE ... with ubidots TOKEN
            static uint8_t atbuf[16]= {0};
            uint16_t ubuflen = sizeof(ubidotsbuf);
            sprintf(atbuf, "AT+CIPSEND=%d\r\n", ubuflen);
            HAL_UART_Transmit(&huart1, atbuf, strlen(atbuf), 100);
            HAL_Delay(200);
            HAL_UART_Transmit(&huart1, ubidotsbuf, ubuflen, 100);
            HAL_Delay(200);;
            memset(atbuf, 0, 16);
            count = 0;
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin == GPIO_PIN_13){
    	if(!espInited){
    		espInited = 1;
    	}
    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1){
        if (++u1RxCnt == BUFSZ)
            u1RxCnt = 0;
        HAL_UART_Receive_IT(&huart1, &u1RxBuf[u1RxCnt], 1);
    }
    if (huart->Instance == USART2){
        if (++u2RxCnt == BUFSZ)
            u2RxCnt = 0;
        HAL_UART_Receive_IT(&huart2, &u2RxBuf[u2RxCnt], 1);
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
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
