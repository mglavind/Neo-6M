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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	HAL_StatusTypeDef ret = HAL_OK; // This is for use in a Clear buffer function, to check if Uart is ready.
	char cmd[100] = "command \r\n";
	char TermMsg[100] = "Message to terminal \r\n";
	uint8_t RecievedData[700] = {0};
	uint8_t c[2] = {0};



	float WakeupTimebase = 16.0 / 32.0; 	// or (float)16 / (float)32
	float SleepTimeSeconds = 60.0; 		// Here the sleep time is defined in seconds
	float WakeUpCounter; 					// The counter is defined, but cannot be calculated before main()
	char WakeUpCounterHex[10]; 			// The hex string for the counter is defined
	uint32_t WakeUpCounterUint;




	#define MAX_MSG_LENGTH 100

	typedef struct {
		float altitude;
		float latitude;
		char latitudeArea;
		float longitude;
		char longitudeArea;
		int fix;
	} GPSData;


	char buffer[50];
	int len;

	int GoToSleep = 0;
	int DataReadyForTx = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void parseNMEA(char* msg, GPSData* data);
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
	WakeUpCounter = SleepTimeSeconds / WakeupTimebase; 	// Wakeup counter is calculated now
	WakeUpCounterUint = (uint32_t)WakeUpCounter; // The Wakeup counter is converted to hex

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  void ClearBuffer(void);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  GPSData gpsData; // Laver et instance af vores struct




// When waking up
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET)
    {
  	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);  // clear the flag

  	  /** display  the string **/
  	  char *str = "Wakeup from the STANDBY MODE\n\n";
  	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

  	  /** Blink the LED **/
  	  for (int i=0; i<20; i++)
  	  {
  		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  		  HAL_Delay(200);
  	  }

  	  /** Disable the WWAKEUP PIN **/
  	  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);  // disable PA0

  	  /** Deactivate the RTC wakeup  **/
  	  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    }





  ClearBuffer();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //HAL_Delay(3000);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)". ", 2, 10);
	  //ClearBuffer();
	  //HAL_UART_Receive_IT(&huart1, RecievedData, 700);


	  if (gpsData.fix){
		  // if fix = 1, har vi en position, og vi kan sende vores position
		  // Power down af GPS?
		  char *str = "We got a fix! \r\n";
		  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
		  // Handle Lora, Sæt GoToSleep til 1, hvis success med Tx.

	  }

	  if (GoToSleep) {
		  EnterStandbyMode();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin_Pin */
  GPIO_InitStruct.Pin = B1_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_Pin_GPIO_Port, &GPIO_InitStruct);

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

void parseNMEA(char* msg, GPSData* data) {
    char* token;
    token = strtok(msg, ",");
    int count = 0;

    // Check the first token for "$GPGGA"
    if (strcmp(token, "$GPGGA") != 0) {
        len = snprintf(buffer, sizeof(buffer), "unknown NMEA message \n");
        HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
        return;
    }

    while (token != NULL) {
        count++;
        switch (count) {
            case 2:
                // Process altitude value
                data->altitude = atof(token);
                break;
            case 3:
                // Process latitude value
                data->latitude = atof(token);
                break;
            case 4:
                // Process latitude area value
                data->latitudeArea = token[0];
                break;
            case 5:
                // Process longitude value
                data->longitude = atof(token);
                break;
            case 6:
                // Process longitude area value
                data->longitudeArea = token[0];
                break;
            case 7:
                // Process fix value
                data->fix = atoi(token);
                break;
        }
        token = strtok(NULL, ",");
    }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	// This has been soft defined in HAL, and can be redefined here.
	HAL_UART_Receive_IT(&huart1, RecievedData, 300);
	HAL_UART_Transmit(&huart2, (uint8_t*)"Interrupt received data: ", 25, 100);
	HAL_UART_Transmit(&huart2, RecievedData,  strlen(RecievedData), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\r\n", 2, 100);


	GPSData gpsData; // Laver et instance af vores struct
	parseNMEA(RecievedData, &gpsData);
	// Access the parsed values
	// Altitude
	len = snprintf(buffer, sizeof(buffer), "Altitude: %.2f\r\n", gpsData.altitude);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	// Latitude
	len = snprintf(buffer, sizeof(buffer), "Latitude: %.5f\r\n", gpsData.latitude);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	// Latitude Area
	len = snprintf(buffer, sizeof(buffer), "Latitude Area: %c\r\n", gpsData.latitudeArea);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	// Longitude
	len = snprintf(buffer, sizeof(buffer), "Longitude: %.5f\r\n", gpsData.longitude);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	// Longitude Area
	len = snprintf(buffer, sizeof(buffer), "Longitude Area: %c\r\n", gpsData.longitudeArea);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);

	// Fix
	len = snprintf(buffer, sizeof(buffer), "Fix: %d\r\n", gpsData.fix);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	memset(RecievedData,0, 30);
	HAL_UART_Receive_IT (&huart1, RecievedData, 30);		//re-starting interrupt
	}

void ClearBuffer(void){
	// Clear RN2483 UART buffer before main code execution.
	// HAL_TIMEOUT is received when HAL_UART_Receive is  not done receiving.
	// huart->TxXferCount will then contain the value of bytes left to be received.
	// If size of received data is less than 1 byte (= no data), HAL_TIMEOUT will be returned
	ret = HAL_OK;
	while (ret != HAL_TIMEOUT) ret = HAL_UART_Receive(&huart1, c, 1, 100);	//RN2483 serial
}



void EnterStandbyMode(void) {

    /** Now enter the standby mode **/
     /* Clear the WU FLAG */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

     /* clear the RTC Wake UP (WU) flag */
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

     /* Display the string */
    char *str = "About to enter the STANDBY MODE\n\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

     /* Blink the LED */
    for (int i=0; i<5; i++)
    {
  	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  	  HAL_Delay(750);
    }

     /* Enable the WAKEUP PIN */
    //HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, WakeUpCounterUint, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
    {
      Error_Handler();
    }

     /* one last string to be sure */
    char *str2 = "STANDBY MODE is ON\n\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);

     /* Finally enter the standby mode */
    HAL_PWR_EnterSTANDBYMode();

}

void SendCommand(void)
{
	HAL_Delay(20);
	ClearBuffer();
	memset(RecievedData,0, 30);
	memset(cmd, 0, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"Command: ", 10, 100);
	char cmd[100] = "$GPGLL\r\n";	// Get and print firmware version
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_UART_Receive(&huart1, RecievedData, 200, 2000);
	HAL_UART_Transmit(&huart2, RecievedData, strlen(RecievedData), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\r\n", 2, 100);
}
void GetSatInfo(void)
{
	HAL_Delay(20);
	ClearBuffer();
	memset(RecievedData,0, 30);
	memset(cmd, 0, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"Command: ", 10, 100);
	char cmd[100] = "$PSTDUMPALMANAC\r\n";	// Get and print firmware version
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
	HAL_UART_Receive(&huart1, RecievedData, 200, 2000);
	HAL_UART_Transmit(&huart2, RecievedData, strlen(RecievedData), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n\r\n", 2, 100);
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
