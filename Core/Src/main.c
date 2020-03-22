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
  *
  *  Created on: 17.02.2019
  *      Author: Mateusz Salamon
  *    www.msalamon.pl
  *
  *      Website: https://msalamon.pl/a-jak-to-jest-z-tym-rtc-na-stm32f4/
  *      GitHub:  https://github.com/lamik/RTC_F401
  *      Contact: mateusz@msalamon.pl
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BACKUP_COUNT 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RTC_TimeTypeDef RtcTime;
RTC_DateTypeDef RtcDate;
uint16_t Milliseconds;

RTC_TimeTypeDef RtcTimeStamp;
RTC_DateTypeDef RtcDateStamp;
uint16_t MillisecondsStamp;

uint8_t CompareSeconds;
uint8_t CompareDate;

uint8_t Message[64];
uint8_t MessageLen;

volatile uint8_t TimeStampFlag;

/* Backup registers table */
uint32_t aBKPDataReg[BACKUP_COUNT] =
{
  RTC_BKP_DR0,  RTC_BKP_DR1,  RTC_BKP_DR2,
  RTC_BKP_DR3,  RTC_BKP_DR4,  RTC_BKP_DR5,
  RTC_BKP_DR6,  RTC_BKP_DR7,  RTC_BKP_DR8,
  RTC_BKP_DR9,  RTC_BKP_DR10, RTC_BKP_DR11,
  RTC_BKP_DR12, RTC_BKP_DR13, RTC_BKP_DR14,
  RTC_BKP_DR15, RTC_BKP_DR16, RTC_BKP_DR17,
  RTC_BKP_DR18, RTC_BKP_DR19
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetRTC(void);
void BackupDateToBR(void);
void RTC_TimeStampSet(void);
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  RTC_TimeStampSet();
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);
	  Milliseconds = ((RtcTime.SecondFraction-RtcTime.SubSeconds)/((float)RtcTime.SecondFraction+1) * 100);
	  HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

	  if(RtcTime.Seconds != CompareSeconds)
	  {
		  MessageLen = sprintf((char*)Message, "Date: %02d.%02d.20%02d Time: %02d:%02d:%02d:%02d\n\r", RtcDate.Date, RtcDate.Month, RtcDate.Year, RtcTime.Hours, RtcTime.Minutes, RtcTime.Seconds, Milliseconds);
		  HAL_UART_Transmit(&huart2, Message, MessageLen, 100);
		  CompareSeconds = RtcTime.Seconds;
	  }

	  if(TimeStampFlag == 1)
	  {
		  MessageLen = sprintf((char*)Message, "TimeStamp! Date: %02d.%02d.20%02d Time: %02d:%02d:%02d:%02d\n\r", RtcDateStamp.Date, RtcDateStamp.Month,
				  RtcDateStamp.Year, RtcTimeStamp.Hours, RtcTimeStamp.Minutes, RtcTimeStamp.Seconds, MillisecondsStamp);
		  HAL_UART_Transmit(&huart2, Message, MessageLen, 100);

		  TimeStampFlag = 0;
	  }

	  if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(TEST_GPIO_Port, TEST_Pin))
	  {
		 while(GPIO_PIN_RESET == HAL_GPIO_ReadPin(TEST_GPIO_Port, TEST_Pin))
		 {}
		 SetRTC();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void BackupDateToBR(void)
{
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, ((RtcDate.Date << 8) | (RtcDate.Month)));
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, ((RtcDate.Year << 8) | (RtcDate.WeekDay)));
}

void SetRTC(void)
{
	RTC_TimeTypeDef sTime = {0};
	RTC_DateTypeDef DateToUpdate = {0};

	/** Initialize RTC and set the Time and Date
	*/
	sTime.Hours = 23;
	sTime.Minutes = 59;
	sTime.Seconds = 56;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
	  Error_Handler();
	}

	DateToUpdate.WeekDay = RTC_WEEKDAY_SATURDAY;
	DateToUpdate.Month = RTC_MONTH_FEBRUARY;
	DateToUpdate.Date = 3;
	DateToUpdate.Year = 20;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	{
	  Error_Handler();
	}

	BackupDateToBR();
}

void RTC_TimeStampSet(void)
{
	/** Enable the TimeStamp
	*/
	if (HAL_RTCEx_SetTimeStamp_IT(&hrtc, RTC_TIMESTAMPEDGE_FALLING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
	{
	Error_Handler();
	}
}
/**
  * @brief  TimeStamp callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
void HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc)
{
  HAL_RTCEx_GetTimeStamp(hrtc, &RtcTimeStamp, &RtcDateStamp, RTC_FORMAT_BIN);
  MillisecondsStamp = ((RtcTime.SecondFraction-RtcTimeStamp.SubSeconds)/((float)RtcTime.SecondFraction+1) * 100);
	TimeStampFlag = 1;
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
