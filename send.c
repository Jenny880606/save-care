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
#include <math.h>
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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t data_rec[6];
int16_t x,y,z;
float xg,yg,zg;

uint8_t u1RxBuf[BUFSZ] = {0};
uint8_t u2RxBuf[BUFSZ] = {0};
volatile uint16_t u1RxCnt = 0;
volatile uint16_t u1TxCnt = 0;
volatile uint16_t u2RxCnt = 0;
volatile uint16_t u2TxCnt = 0;
volatile uint8_t atret = 0;
volatile uint8_t espInited = 0;
volatile uint8_t val = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_write(uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|0x40; // multibyte write
	data[1] = value;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, data, 2, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
void adxl_read(uint8_t address)
{
	address |= 0x80; // read operation
	address |= 0x40; // multibyte read
	uint8_t rec = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &address, 1, 100);
	HAL_SPI_Receive(&hspi3, data_rec, 6, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
uint8_t adxl_read_devid(void)
{
	uint8_t address = 0x00 | 0x80; // read operation
	uint8_t value = 0;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, &address, 1, 100);
	HAL_SPI_Receive(&hspi3, &value, 1, 100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	return value;
}
void adxl_init(void)
{
	adxl_write(0x31, 0x01); // data_format range= +- 4g
	adxl_write(0x2d, 0x00); // reset all bits
	adxl_write(0x2d, 0x08); // power_cntl measure and wake up 8hz
}

void InitESP8266(){
    HAL_UART_Transmit(&huart1, "AT+CWMODE=3\r\n", 13, 100);
    HAL_Delay(3000);
    HAL_UART_Transmit(&huart1, "AT+CWJAP=\"yaya\",\"yayayaya\"\r\n", 35, 100); // ADD WiFi AP ssid and password
    HAL_Delay(10000);
    HAL_UART_Transmit(&huart1, "AT+CIPSTART=\"TCP\",\"things.ubidots.com\",9012\r\n", 45, 100);
    HAL_Delay(3000);
    HAL_UART_Transmit(&huart1, "ATE0\r\n", 6, 100);
    HAL_Delay(3000);
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
        /*
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
                    char *delim = strstr(ipdcmd, ":");
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
        */
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
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  uint8_t devid = adxl_read_devid();
  if(devid != 0xE5){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    return 1;
  }
  adxl_init();

	HAL_UART_Receive_IT(&huart1, &u1RxBuf[u1RxCnt], 1);
	HAL_UART_Receive_IT(&huart2, &u2RxBuf[u2RxCnt], 1);
	HAL_Delay(1000);
//    InitESP8266();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t outputStr[80]={0};
    uint8_t count = 0;

    while (1)
    {
        processUart1Received();
        processUart2Received();
        HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        adxl_read(0x32);
        	  x = ((data_rec[1] << 8) | data_rec[0]);
        	  y = ((data_rec[3] << 8) | data_rec[2]);
        	  z = ((data_rec[5] << 8) | data_rec[4]);
        	  xg = x * 0.0078;
        	  yg = y * 0.0078;
        	  zg = z * 0.0078;
        	  //sprintf(outputStr, "ax = %f, ay = %f, az = %f\r\n\0", xg, yg, zg);
        	  sprintf(outputStr, "average = %f\r\n\0",sqrt(xg*xg + yg*yg + zg*zg));
        	  HAL_UART_Transmit(&huart2, outputStr, strlen(outputStr), 0xFFFF);
        	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        	  HAL_Delay(100);

        if(!espInited)
        	continue;
        if(espInited == 1){
        	InitESP8266();
        	espInited = 2;
        }
        else if(count++ > 9){

            static uint8_t ubidotsbuf[BUFSZ] = {0};
            static uint8_t atbuf[16]= {0};
            if(sqrt(xg*xg + yg*yg + zg*zg)>1.5)
            	val = 0;
            sprintf(ubidotsbuf,
            "STM32/1.0|POST|BBFF-08q19mQ1vYLAzQSeKiCxyr7XKTdIgf|123:stm32=>val:%d|end", val ); // REPLACE ... with ubidots TOKEN
            uint16_t ubuflen = strlen(ubidotsbuf);
            sprintf(atbuf, "AT+CIPSEND=%d\r\n", ubuflen);
            HAL_UART_Transmit(&huart1, atbuf, strlen(atbuf), 100);
            HAL_Delay(200);
            HAL_UART_Transmit(&huart1, ubidotsbuf, ubuflen, 100);
            memset(atbuf, 0, 16);
            memset(ubidotsbuf, 0, ubuflen);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1496;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 254;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
    if(GPIO_Pin == GPIO_PIN_4){
    	val = 1;
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

uint16_t tones[7]={1496, 1333, 1187, 1123, 999, 890, 793};
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(val == 0)
	{
		if (htim->Instance == htim1.Instance)
		{
			// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			static int counter = 0;
			if(counter >= 8){
				TIM2->CCR2 = 0;
			}else{
				TIM2->PSC = tones[counter++];
				TIM2->CCR2 = 127;
			}
		}
	}
}/**/
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
