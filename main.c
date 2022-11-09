/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include "dwt.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STANDBY_ON				true
#define STANDBY_OFF				false
#define BIT8_MODE				true
#define BIT6_MODE				false
#define DISPLAY_ON				true
#define DISPLAY_OFF				false
#define X_DOWN					0
#define X_UP					1
#define Y_DOWN					2
#define Y_UP					3
#define TRANSFER_INSTRUCTION	true
#define TRANSFER_DATA			false
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
const uint16_t aPinData[8] = { D0_Pin
	, D1_Pin
	, D2_Pin
	, D3_Pin
	, D4_Pin
	, D5_Pin
	, D6_Pin
	, D7_Pin
};
__IO static bool bCeIsLow = false;
__IO static bool bDataAvailable = false;

int8_t aMemory[64][12] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void doTransfer(const bool);// transfer data/instruction from MCU to LCD
void _86E(const bool);		// word length 8/6 bit
void DPE(const bool);		// display on/off
void UDE(const uint8_t);	// counter select/mode select
void SCE(const uint8_t);	// contrast set
void OPA1(const uint8_t);	// opamp1 power setting
void OPA2(const uint8_t);	// opamp2 power setting
void SXE(const uint8_t);	// x-address set
void SYE(const uint8_t);	// y-address set
void SZE(const uint8_t);	// z-address set
void DAWR(const int8_t);	// display data write
void reset();				// reset LCD controller to default
void standby(const bool);	// standby mode on/off

void transferMemory();
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
  DWT_Delay_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ////////////////////////////////////////////////////////////////////////////
  // start timer 4
  HAL_TIM_Base_Start_IT(&htim4);
  reset();
  standby(STANDBY_OFF);
  // CE high
  HAL_GPIO_WritePin(GPIOC, ChipEnable_Pin, GPIO_PIN_SET);
  // 8-bit mode
  _86E(BIT8_MODE);
  // counter/mode select
  UDE(3);
  // set power supplying ability of opamp1 and opamp2
  OPA1(3);
  OPA2(2);
  // set contrast
  SCE(0x36);
  // display on
  DPE(DISPLAY_ON);

  // fill memory with pattern
  uint8_t x8 = 0;
  for (uint8_t x = 0; x < 64; x++)
  {
	  x8 = x / 8;
	  for (uint8_t y = 0; y < 12; y++)
	  {
		  if ((((x8 % 2) == 0) && ((y % 2) == 0)) ||
			  (((x8 % 2) == 1) && ((y % 2) == 1)))
		  {
			  aMemory[x][y] = 0;
		  }
		  else
		  {
			  aMemory[x][y] = 0xFF;
		  }
	  }
  }
  ////////////////////////////////////////////////////////////////////////////

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	transferMemory();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 4 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100 - 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Reset_Pin|Standby_Pin|DataOrInstruction_Pin|ChipEnable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D0_Pin|D1_Pin|D6_Pin|D7_Pin
                          |D2_Pin|D3_Pin|D4_Pin|D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ReadOrWrite_GPIO_Port, ReadOrWrite_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Reset_Pin Standby_Pin DataOrInstruction_Pin ChipEnable_Pin */
  GPIO_InitStruct.Pin = Reset_Pin|Standby_Pin|DataOrInstruction_Pin|ChipEnable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : D0_Pin D1_Pin D6_Pin D7_Pin
                           D2_Pin D3_Pin D4_Pin D5_Pin */
  GPIO_InitStruct.Pin = D0_Pin|D1_Pin|D6_Pin|D7_Pin
                          |D2_Pin|D3_Pin|D4_Pin|D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ReadOrWrite_Pin */
  GPIO_InitStruct.Pin = ReadOrWrite_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ReadOrWrite_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
////////////////////////////////////////////////////////////////////////////
//****************************************************************************
//*                     HAL_TIM_PeriodElapsedCallback
//****************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (bCeIsLow)
	{
		// CE high
		HAL_GPIO_WritePin(GPIOC, ChipEnable_Pin, GPIO_PIN_SET);
		bCeIsLow = false;
		bDataAvailable = false;
	}
	else
	{
		if (bDataAvailable)
		{
			// CE low
			HAL_GPIO_WritePin(GPIOC, ChipEnable_Pin, GPIO_PIN_RESET);
			bCeIsLow = true;
		}
	}
}
//****************************************************************************
//*                     doTransfer
//*
//* bInstruction: TRANSFER_INSTRUCTION=true | TRANSFER_DATA=false
//****************************************************************************
void doTransfer(const bool bInstruction)
{
	if (bInstruction)
	{
		// D/I low
		HAL_GPIO_WritePin(GPIOC, DataOrInstruction_Pin, GPIO_PIN_RESET);
	}
	else
	{
		// D/I high
		HAL_GPIO_WritePin(GPIOC, DataOrInstruction_Pin, GPIO_PIN_SET);
	}
	// R/W low
	HAL_GPIO_WritePin(GPIOB, ReadOrWrite_Pin, GPIO_PIN_RESET);

	// block while Chip Enable (CE) is low
	while (1)
	{
		DWT_Delay_us(10);
		if (!bCeIsLow) break;
	}
	bDataAvailable = true;
}
//****************************************************************************
//*                     _86E
//*
//* word length 8/6 bit
//* _8bits: BIT8_MODE=true (1) | BIT6_MODE=false (0)
//* D0: 1=8bit, 0=6bit
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  0  0  0  0  0  0  m
//****************************************************************************
void _86E(const bool _8Bits)
{
	if (_8Bits)
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[0], GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[0], GPIO_PIN_RESET);
	}
	// set remaining bit
	for (uint8_t i = 1; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
	}
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     DPE
//*
//* display on/off
//* bDisplayOn: DISPLAY_ON=true (1) | DISPLAY_OFF=false (0)
//* D0: 1=on, 0=off
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  0  0  0  0  0  1  x
//****************************************************************************
void DPE(const bool bDisplayOn)
{
	if (bDisplayOn)
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[0], GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[0], GPIO_PIN_RESET);
	}
	// D1 high
	HAL_GPIO_WritePin(GPIOA, aPinData[1], GPIO_PIN_SET);
	// set remaining bit
	for (uint8_t i = 2; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
	}
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     UDE
//*
//* counter/mode select
//* select: X_DOWN (0) | X_UP (1) | Y_DOWN (2) | Y_UP (3)
//* mode...... D0: 0=down, 1=up
//* counter... D1: 0=X, 1=Y
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  0  0  0  0  1  c  m
//****************************************************************************
void UDE(const uint8_t select)
{
	int8_t mask = 1;
	uint8_t i = 0;
	for (; i < 2; i++)
	{
		if (select & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// D2 high
	HAL_GPIO_WritePin(GPIOA, aPinData[i++], GPIO_PIN_SET);
	// set remaining data bit
	for (; i < 8; i++)
	{
		HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
	}
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     SCE
//*
//* contrast set
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  1  1  c  c  c  c  c  c
//****************************************************************************
void SCE(const uint8_t contrast_control)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 6; i++)
	{
		if (contrast_control & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// set remaining data bit
	HAL_GPIO_WritePin(GPIOA, aPinData[6], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, aPinData[7], GPIO_PIN_SET);
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     OPA1
//*
//* set the power supplying ability of opamp1
//* val is a 3-bit value
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  0  0  1  0  v  v  v
//****************************************************************************
void OPA1(const uint8_t val)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 3; i++)
	{
		if (val & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// set remaining data bit
	HAL_GPIO_WritePin(GPIOA, aPinData[3], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[4], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, aPinData[5], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[6], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[7], GPIO_PIN_RESET);
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     OPA2
//*
//* set the power supplying ability of opamp1
//* val is a 2-bit value
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  0  0  0  1  0  v  v
//****************************************************************************
void OPA2(const uint8_t val)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 2; i++)
	{
		if (val & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// set remaining data bit
	HAL_GPIO_WritePin(GPIOA, aPinData[2], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[3], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, aPinData[4], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[5], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[6], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[7], GPIO_PIN_RESET);
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     SXE
//*
//* x-address set
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  1  0  x  x  x  x  x  x
//****************************************************************************
void SXE(const uint8_t x)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 6; i++)
	{
		if (x & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// set remaining data bit
	HAL_GPIO_WritePin(GPIOA, aPinData[6], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[7], GPIO_PIN_SET);
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     SYE
//*
//* y-address set
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  0  1  y  y  y  y  y
//****************************************************************************
void SYE(const uint8_t y)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 5; i++)
	{
		if (y & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// set remaining data bit
	HAL_GPIO_WritePin(GPIOA, aPinData[5], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, aPinData[6], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, aPinData[7], GPIO_PIN_RESET);
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     SZE
//*
//* z-address set
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   0   0  0  1  z  z  z  z  z  z
//****************************************************************************
void SZE(const uint8_t z)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 6; i++)
	{
		if (z & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// set remaining data bit
	HAL_GPIO_WritePin(GPIOA, aPinData[6], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, aPinData[7], GPIO_PIN_RESET);
	// transfer instruction from MCU to LCD controller
	doTransfer(TRANSFER_INSTRUCTION);
}
//****************************************************************************
//*                     DAWR
//*
//* display data write
//* D/I R/W D7 D6 D5 D4 D3 D2 D1 D0
//*   1   0  x  x  x  x  x  x  x  x
//****************************************************************************
void DAWR(const int8_t data)
{
	int8_t mask = 1;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (data & mask)
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, aPinData[i], GPIO_PIN_RESET);
		}
		mask <<= 1;
	}
	// transfer data from MCU to LCD controller
	doTransfer(TRANSFER_DATA);
}
//****************************************************************************
//*                     reset
//*
//* reset LCD controller to default
//* bring reset (RST) down for 1 milliseconds
//*
//* display............ OFF
//* word length........ 8 bit/ word
//* counter mode....... Y counter / up mode
//* Y (Page)-address... Page=0
//* X-address.......... Xad=0
//* Z-address.......... Zad=0
//* OP-Amp............. ON
//* OPA1............... Min.
//* OPA2............... Min.
//* CONTRAST........... Min. (Vlc5=Vee1,2)
//*
//****************************************************************************
void reset()
{
	HAL_GPIO_WritePin(GPIOC, Reset_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, Reset_Pin, GPIO_PIN_SET);
}
//****************************************************************************
//*                     standby
//*
//* standby mode
//* bStandbyOn: false=active, true=standby
//****************************************************************************
void standby(const bool bStandbyOn)
{
	if (bStandbyOn)
	{
		// bring standby (STB) low
		HAL_GPIO_WritePin(GPIOC, Standby_Pin, GPIO_PIN_RESET);
	}
	else
	{
		// bring standby (STB) high
		HAL_GPIO_WritePin(GPIOC, Standby_Pin, GPIO_PIN_SET);
	}
}
//****************************************************************************
//*                     transferMemory
//****************************************************************************
void transferMemory()
{
	SXE(0);
	SYE(0);
	SZE(0);
	for (uint8_t x = 0; x < 64; x++)
	{
		SXE(x);
		for (uint8_t y = 0; y < 12; y++)
		{
			DAWR(aMemory[x][y]);
		}
	}
}
////////////////////////////////////////////////////////////////////////////

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
