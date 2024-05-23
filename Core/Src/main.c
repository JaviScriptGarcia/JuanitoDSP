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
#include <stdint.h>
#include <stdio.h>
#include <limits.h>
#ifndef DSP_H
#include "dsp.h"
#endif
#ifndef GLOBAL_H
#include "global.h"
#endif
// #include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUF_BEGIN 0
#define BUF_HALF (BUFFER_SIZE/2)
#define DMA_INT_PERIOD (float)((BUFFER_SIZE / 2) * 100000 / SAMPLE_RATE) // 10 x us

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

typedef union
{
  float f32;
  int16_t q15;
  int32_t q31;
} tUnionBuf;

typedef enum
{
  F32,
  Q15,
  Q31
} tArithmetic;

// *****************************************************************************
// INPUT, OUTPUT AND PROCESSING BUFFERS
static uint32_t adc[BUFFER_SIZE*2];                  //  RX buffer (Stereo PCM)
static int16_t dac[BUFFER_SIZE*2];                   //  TX buffer (Stereo PCM)
static tUnionBuf general[MAX_CHANNELS][BUFFER_SIZE]; //  Processing buffers
// *****************************************************************************

// *****************************************************************************
// Convolution instances
static tConvq15 convq15[MAX_CONV];
// *****************************************************************************

// *****************************************************************************
// FIR instances
static tFIRf32 FIRf32[MAX_FILTERS];
static tFIRq15 FIRq15[MAX_FILTERS];

// FIR arm_math.h instances
static arm_fir_instance_f32 insFIRf32[MAX_FILTERS];
static arm_fir_instance_q15 insFIRq15[MAX_FILTERS];
// *****************************************************************************

// *****************************************************************************
// IIR 2nd order biquads
static tIIRf32 IIRf32[MAX_FILTERS];    // IIR 2nd order instance
static tIIRq15 IIRq15[MAX_FILTERS];    // IIR 2nd order instance
static tIIRq31 IIRq31[MAX_FILTERS];    // IIR 2nd order instance
static tParamConfig filterConfig[MAX_FILTERS]; // IIR 2nd order config params
static tGain normGain[MAX_CHANNELS]; // Anti saturation pre-filter gain

// IIR arm_math.h 2nd order biquads pointer instance
static arm_biquad_cascade_df2T_instance_f32 insIIRf32[MAX_FILTERS];
static arm_biquad_casd_df1_inst_q15 insIIRq15[MAX_FILTERS];
static arm_biquad_casd_df1_inst_q31 insIIRq31[MAX_FILTERS];
// *****************************************************************************

// *****************************************************************************
// APP LOGIC STATE VARIABLES
static tErrorCode appError;
static uint8_t dataReadyFlag = 0; // Buffer state, 0 = empty, 1 = half, 2 = full
static uint8_t filterConfigFlag = 0; // 0 = configured, 1 = new config received
static uint32_t timerCount = 0;   // Performance monitor timer
static float cpuUsage = 0;        // CPU real-time usage (%)
uint16_t nSamples = BUF_HALF;
uint16_t bufSize = BUFFER_SIZE;


// *****************************************************************************

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
static void AppInit(void);
static tErrorCode CheckParams(void);
static tErrorCode ProcessData(void);
static tErrorCode CheckConfig(void);
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
  AppInit();
  DSP_TestFilters(filterConfig);
  
  if (RES_OK != DSP_UpdateIIRInstances(filterConfig, IIRf32, IIRq15, IIRq31, 
  insIIRf32, insIIRq15, insIIRq31, normGain))
  {appError = ERROR_CODE_15; goto error;}

  if (RES_OK != DSP_UpdateFIRInstances(filterConfig, FIRf32, FIRq15, insFIRf32,
                insFIRq15))
  {appError = ERROR_CODE_16; goto error;}

  if (RES_OK != DSP_UpdateConvolutionInstances(convq15))
  {appError = ERROR_CODE_16; goto error;}

  if (RES_OK != CheckParams())
  {appError = ERROR_CODE_17; goto error;}
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_Base_Start_IT(&htim3);

    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*) dac, BUFFER_SIZE * 2);
    HAL_I2S_Receive_DMA(&hi2s2, (uint16_t*) adc, BUFFER_SIZE * 2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
      if (RES_OK != CheckConfig()) goto error;
      if (RES_OK != ProcessData()) goto error;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    error:
      // Toggle an LED to notify error
      Error_Handler();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV16;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV16;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_MSB;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s3.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s3.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s3.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 124-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_VCP_RX_Pin STLK_VCP_TX_Pin */
  GPIO_InitStruct.Pin = STLK_VCP_RX_Pin|STLK_VCP_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// *****************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// *****************************************************************************
// Description: Callback for timers
// Parameters: timer handler
// Returns: nothing
// *****************************************************************************
{
  if(htim->Instance == TIM16)
  {
    timerCount++; // Performance monitor increase, period = 10 uS
    // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);         // Toggle LED red
  }
  if(htim->Instance == TIM3)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);            // Toggle LED red
  }
}

// *****************************************************************************
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef* hi2s2)
// *****************************************************************************
// Description: Callback for half buffer completion
// Parameters: i2s handle
// Returns: nothing
// *****************************************************************************
{
    HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);   // Toggle LED amarillo
    if (0 == dataReadyFlag) timerCount = 0; // Performance monitor reset timer
    dataReadyFlag = 1;
}

// *****************************************************************************
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef* hi2s2)
// *****************************************************************************
// Description: Callback for full buffer completion
// Parameters: i2s handle
// Returns: nothing
// *****************************************************************************
{
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);     // Toggle LED verde
    if (0 == dataReadyFlag) timerCount = 0; // Performance monitor reset timer
    dataReadyFlag = 2;
}

// *****************************************************************************
static void AppInit(void)
// *****************************************************************************
// Description: Initializes structure values with 0
// Parameters: none
// Returns: nothing
// *****************************************************************************
{
  uint16_t i, j;
  
  for (i = 0; i<MAX_FILTERS; i++)
  {
    // Init delayed samples for f32
    for (j = 0; j < 2; j++) IIRf32[i].delays[j] = 0;
    // Init biquad coeffs for f32
    for (j = 0; j < 5; j++) IIRf32[i].coeffs[j] = 0;
    arm_biquad_cascade_df2T_init_f32(insIIRf32, 1, IIRf32[i].coeffs, 
                                     IIRf32[i].delays);

    // Init delayed samples for q15
    for (j = 0; j < 4; j++) IIRq15[i].delays[j] = 0;
    // Init biquad coeffs for q15
    for (j = 0; j < 6; j++) IIRq15[i].coeffs[j] = 0;
    arm_biquad_cascade_df1_init_q15(insIIRq15, 1, IIRq15[i].coeffs, 
                                    IIRq15[i].delays, 0); 

    // Init delayed samples for q31
    for (j = 0; j < 4; j++) IIRq31[i].delays[j] = 0;
    // Init biquad coeffs for q31
    for (j = 0; j < 5; j++) IIRq31[i].coeffs[j] = 0;
    arm_biquad_cascade_df1_init_q31(insIIRq31, 1, IIRq31[i].coeffs, 
                                    IIRq31[i].delays, 0); 

    // Init channel routing
    IIRf32[i].channel = CHANNEL_NONE;
    IIRq15[i].channel = CHANNEL_NONE;
    IIRq15[i].channel = CHANNEL_NONE;
    FIRf32[i].channel = CHANNEL_NONE;
    FIRq15[i].channel = CHANNEL_NONE;

    filterConfig[i].channel = CHANNEL_NONE;
    filterConfig[i].gain = 0;
    normGain[i].f32 = 0;
    normGain[i].q31 = 0;
    normGain[i].q15 = 0;
  }

  // Init convolution channels
  for (i = 0; i < MAX_CONV; i++) convq15[i].channel = CHANNEL_NONE;
}

// *****************************************************************************
static tErrorCode CheckParams(void)
// *****************************************************************************
// Description: Checks system configuration to ensure compatibility
// Parameters: None.
// Returns: Error code.
// *****************************************************************************
{
  if ((4 < DAC_QUANTITY) || (0 > DAC_QUANTITY)) return RES_ERROR_PARAM;
  if ((4 < ADC_QUANTITY) || (0 > ADC_QUANTITY)) return RES_ERROR_PARAM;
  if (0 != (BUFFER_SIZE % 4)) return RES_ERROR_PARAM; // Must be a multiple of 4

  return RES_OK;
}

// *****************************************************************************
static tErrorCode NormalizeChannels(void *inBuf, tGain *gain, 
                                    tArithmetic bufType)
// *****************************************************************************
// Description: Normalizes all channels with their corresponding gain to avoid
// filter saturation.
// Parameters: 
// Returns: 
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == gain)) 
  {appError = ERROR_CODE_12; return RES_ERROR_PARAM;}

  uint8_t i;
  size_t sizeDif = 0;

  switch (bufType)
  {
    case Q15:
      sizeDif = sizeof(int32_t)/sizeof(int16_t);
      for(i = CHANNEL_0; i < MAX_CHANNELS; i++)
      {
        if (SHRT_MAX == gain->q15) break;
        if (RES_OK != DSP_Gain_q15_arm((int16_t *)inBuf + (bufSize * i) 
                      * sizeDif, gain->q15, gain->postShift, nSamples))
        {appError = ERROR_CODE_13; return RES_ERROR;}
      }
      break;
    
    case Q31:
      sizeDif = sizeof(int32_t)/sizeof(int32_t);
      for(i = CHANNEL_0; i < MAX_CHANNELS; i++)
      {
        if (LONG_MAX == gain->q31) break;
        if (RES_OK != DSP_Gain_q31_arm((int32_t *)inBuf + (bufSize * i) 
                      * sizeDif, gain->q31, gain->postShift, nSamples))
        {appError = ERROR_CODE_13; return RES_ERROR;}
      }
      break;
    
    case F32:
      if ((float)1 == gain->f32) break;
      sizeDif = sizeof(int32_t)/sizeof(float);
      for(i = CHANNEL_0; i < MAX_CHANNELS; i++)
      {
        if (RES_OK != DSP_Gain_f32_arm((float *)inBuf + (bufSize * i) 
                      * sizeDif, gain->f32, nSamples))
        {appError = ERROR_CODE_13; return RES_ERROR;}
      }
      break;

    default: 
      appError = ERROR_CODE_12; 
      return RES_ERROR_PARAM;
      break;

    gain++;
  }

  return RES_OK;
}

// *****************************************************************************
static tErrorCode ApplyFilters(void *inBuf, tFIRf32 *pFIRf32, tFIRq15 *pFIRq15, 
                               arm_fir_instance_f32 *f, arm_fir_instance_q15 *g,
                               tIIRf32 *pIIRf32, 
                               tIIRq15 *pIIRq15, 
                               tIIRq31 *pIIRq31,
                               arm_biquad_cascade_df2T_instance_f32 *s,
                               arm_biquad_casd_df1_inst_q15 *q, 
                               arm_biquad_casd_df1_inst_q31 *r,
                               tArithmetic bufType)
// *****************************************************************************
// Description: Processes multiple channels with desired filters and desired
// arithmetic.
// Parameters: 
//   *inBuf: Pointer to the buffer storing all channels
//   *pIIRf32: Pointer to the F32 IIR filters
//   *pIIRq15: Pointer to the Q15 IIR filters
//   *inst131: Pointer to the Q31 IIR filters
//   *s: Pointer to the F32 IIR arm_math filter instances
//   *q: Pointer to the Q15 IIR arm_math filter instances
//   *r: Pointer to the Q31 IIR arm_math filter instances
//   nSamples: Number of array positions to be filtered
//   bufType: Type of aritmethic to use, must match the channel variable type
// Returns: error code
// *****************************************************************************
{
  if ((NULL == pIIRf32) || (NULL == pIIRq15) || (NULL == pIIRq31) ||
      (NULL == inBuf)) 
  {appError = ERROR_CODE_9; return RES_ERROR_PARAM;}
  
  uint8_t i;
  uint16_t filterChan = 0;
  size_t sizeDif = 0; // Difference between largest and used union member size

  switch (bufType)
  {
    case Q15:
      sizeDif = sizeof(int32_t)/sizeof(int16_t);
      for (i = CHANNEL_0; i < MAX_FILTERS; i++)
      {
        if (CHANNEL_NONE != pIIRq15->channel)
        {
          filterChan = pIIRq15->channel;
          #ifdef USE_LIBRARY
          if (RES_OK != DSP_IIR_q15_arm((int16_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, q))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #else
          if (RES_OK != DSP_IIR_q15((int16_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, pIIRq15))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #endif
        }
        if (CHANNEL_NONE != pFIRq15->channel)
        {
          filterChan = pFIRq15->channel;
          #ifdef USE_LIBRARY
          if (RES_OK != DSP_FIR_q15_arm((int16_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, g))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #else
          if (RES_OK != DSP_FIR_q15((int16_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, pFIRq15))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #endif
        }
        pIIRq15++;
        q++;
        pFIRq15++;
        g++;
      }
      break;

    case Q31:
      sizeDif = sizeof(int32_t)/sizeof(int32_t);
      for (i = 0; i < MAX_FILTERS; i++)
      {
        if (CHANNEL_NONE != pIIRq31->channel)
        {
          filterChan = pIIRq31->channel;
          #ifdef USE_LIBRARY
          if (RES_OK != DSP_IIR_q31_arm((int32_t *)inBuf + (filterChan *
                        bufSize) * sizeDif, nSamples, r))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #else
          if (RES_OK != DSP_IIR_q31((int32_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, pIIRq31))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #endif
        }
        r++;
        pIIRq31++;
      }
      break;
  
    case F32:
      sizeDif = sizeof(int32_t)/sizeof(float);
      for (i = 0; i < MAX_FILTERS; i++)
      {
        if (CHANNEL_NONE != pIIRf32->channel)
        {
          filterChan = pIIRf32->channel;
          #ifdef USE_LIBRARY
          if (RES_OK != DSP_IIR_f32_arm((float *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, s))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #else
          if (RES_OK != DSP_IIR_f32((float *)inBuf + (filterChan * bufSize)
                        * sizeDif, nSamples, pIIRf32))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #endif
        }
        if (CHANNEL_NONE != pFIRf32->channel)
        {
          filterChan = pFIRf32->channel;
          #ifdef USE_LIBRARY
          if (RES_OK != DSP_FIR_f32_arm((float *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, f))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #else
          if (RES_OK != DSP_FIR_f32((float *)inBuf + (filterChan * bufSize)
                        * sizeDif, nSamples, pFIRf32))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #endif
        }

        s++;
        pIIRf32++;
        f++;
        pFIRf32++;
      }
      break;
    
    default: 
      appError = ERROR_CODE_9; 
      return RES_ERROR_PARAM;
      break;
  }

  return RES_OK;
}


// *****************************************************************************
static tErrorCode ApplyConvolution(void *inBuf, tConvq15 *pConvq15, 
                  tArithmetic bufType)
// *****************************************************************************
// Description: Applies convolution as configured.
// Parameters: 
//   *inBuf: Pointer to audio input signal.
//   *pConvq15: Ponter to q15 convolution instance.
//   nSamples: Number of samples to process.
//   bufType: Type of arithmetic to be used.
// Returns: Error code.
// *****************************************************************************
{
  if ((NULL == pConvq15) || (NULL == inBuf)) 
  {appError = ERROR_CODE_18; return RES_ERROR_PARAM;}
  
  uint8_t i;
  uint16_t filterChan = 0;
  size_t sizeDif = 0; // Difference between largest and used union member size

  switch (bufType)
  {
    case Q15:
      sizeDif = sizeof(int32_t)/sizeof(int16_t);
      for (i = CHANNEL_0; i < MAX_CONV; i++)
      {
        if (CHANNEL_NONE != pConvq15->channel)
        {
          filterChan = pConvq15->channel;
          #ifdef USE_LIBRARY
          if (RES_OK != DSP_Convolution_q15_arm((int16_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, pConvq15))
          {appError = ERROR_CODE_19; return RES_ERROR;}
          #else
          if (RES_OK != DSP_Convolution_q15((int16_t *)inBuf + 
                        (filterChan * bufSize) * sizeDif, nSamples, pConvq15))
          {appError = ERROR_CODE_10; return RES_ERROR;}
          #endif
        }
        pConvq15++;
      }
      break;

    case Q31:
      break;
  
    case F32:
      break;
    
    default: 
      appError = ERROR_CODE_18; 
      return RES_ERROR_PARAM;
      break;
  }
  
  return RES_OK;  
}

// *****************************************************************************
static tErrorCode DecodeData(uint32_t *inBuf, void *outBufL, void *outBufR, 
                             tArithmetic bufType)
// *****************************************************************************
// Description: Decodes a buffer of PCM data from the PCM1808 ADC
// Parameters: 
//   *inBuf: Pointer to the buffer containing stereo data sent by the ADC
//   *outBufL: Pointer to the buffer where the left channel data will be stored
//   *outBufR: Pointer to the buffer where the right channel data will be stored
//   nSamples: Number of samples of one mono channel
//   bufType: Type of arithmetic that will be used
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == inBuf) || (NULL == outBufL) || (NULL == outBufR)) 
  {appError = ERROR_CODE_0; return RES_ERROR_PARAM;}

  uint16_t nSamples = BUF_HALF;
  uint16_t bufSize = BUFFER_SIZE;

  switch (bufType)
  {
    case Q15:
      if (RES_OK != DSP_Int24ToInt16(inBuf, bufSize))
      {appError = ERROR_CODE_1; return RES_ERROR;}
    
      if (RES_OK != DSP_DecodePCM_Int16(inBuf, (int16_t *)outBufL, 
                    (int16_t *)outBufR, nSamples))
      {appError = ERROR_CODE_2; return RES_ERROR;}  
      break;

    case Q31:
      if (RES_OK != DSP_Int24ToInt32(inBuf, bufSize))
      {appError = ERROR_CODE_1; return RES_ERROR;}
    
      if (RES_OK != DSP_DecodePCM_Int32(inBuf, (int32_t *)outBufL, 
                    (int32_t *)outBufR, nSamples)) 
      {appError = ERROR_CODE_2; return RES_ERROR;}  
      break;    

    case F32:
      if (RES_OK != DSP_Int24ToInt32(inBuf, bufSize))
      {appError = ERROR_CODE_1; return RES_ERROR;}
    
      if (RES_OK != DSP_DecodePCM_Int32(inBuf, (int32_t *)outBufL, 
                    (int32_t *)outBufR, nSamples))
      {appError = ERROR_CODE_2; return RES_ERROR;}  

      if (RES_OK != DSP_q31_to_f32_arm((int32_t *)outBufL, (float *)outBufL, 
                    nSamples))
      {appError = ERROR_CODE_3; return RES_ERROR;}
      if (RES_OK != DSP_q31_to_f32_arm((int32_t *)outBufR, (float *)outBufR, 
                    nSamples))
      {appError = ERROR_CODE_4; return RES_ERROR;}
      break;

    
    default:
      appError = ERROR_CODE_0;
      return RES_ERROR_PARAM;
      break;
  }
  return RES_OK;
}

// *****************************************************************************
static tErrorCode EncodeData(void *inBufL, void *inBufR, int16_t *outBuf,
                             tArithmetic bufType)
// *****************************************************************************
// Description: Decodes a buffer of PCM data to the PCM5102 DAC
// Parameters: 
//   *inBufL: Pointer to the buffer where the left channel data is stored
//   *inBufR: Pointer to the buffer where the right channel data is stored
//   *outBuf: Pointer to the buffer that will store stereo data for the DAC
//   bufType: Type of arithmetic that will be used
// Returns: Error code
// *****************************************************************************
{
  if ((NULL == outBuf) || (NULL == inBufL) || (NULL == inBufR))
  {appError = ERROR_CODE_5; return RES_ERROR_PARAM;}

  uint16_t nSamples = BUF_HALF;

  switch (bufType)
  {
    case Q15:
      break;
    
    case Q31:
      if (RES_OK != DSP_q31_to_q15_arm((int32_t *)inBufL, (int16_t *)inBufL,
                    nSamples)) 
      {appError = ERROR_CODE_7; return RES_ERROR;}
      if (RES_OK != DSP_q31_to_q15_arm((int32_t *)inBufR, (int16_t *)inBufR, 
                    nSamples))
      {appError = ERROR_CODE_8; return RES_ERROR;}
      break;
  
    case F32:
      if (RES_OK != DSP_f32_to_q15_arm((float *)inBufL, (int16_t *)inBufL,
                    nSamples)) 
      {appError = ERROR_CODE_7; return RES_ERROR;}
      if (RES_OK != DSP_f32_to_q15_arm((float *)inBufR, (int16_t *)inBufR, 
                    nSamples)) 
      {appError = ERROR_CODE_8; return RES_ERROR;}
      break;
  
    default:
      appError = ERROR_CODE_5; 
      return RES_ERROR_PARAM;
      break;
  }

  if (RES_OK != DSP_EncodePCM(outBuf, (int16_t *)inBufL, (int16_t *)inBufR,
                nSamples)) 
  {appError = ERROR_CODE_6; return RES_ERROR;}  

  return RES_OK;
}

// *****************************************************************************
static tErrorCode ProcessData(void)
// *****************************************************************************
// Description: Processsing function. FSM managing application functions.
// Parameters: none
// Returns: error code
// *****************************************************************************
{

  switch (dataReadyFlag)
  {
    case 0: // Buffer still receiving data
      return RES_OK;
      
    case 1: // Buffer half full, process first half
      
      // Decode ADC1 from PCM
      if (RES_OK != DecodeData(&adc[BUF_BEGIN], 
                    &general[CHANNEL_0][BUF_BEGIN],
                    &general[CHANNEL_1][BUF_BEGIN], 
                    Q15)) return RES_ERROR;

      // Normalize channels
      if (RES_OK != NormalizeChannels(&general[CHANNEL_0][BUF_BEGIN], 
                                      &normGain[CHANNEL_0], Q15))
      {return RES_ERROR;}

      // Filter all channels
      if (RES_OK != ApplyFilters(&general[CHANNEL_0][BUF_BEGIN], FIRf32, FIRq15,
                    insFIRf32, insFIRq15, IIRf32, IIRq15, IIRq31, insIIRf32, 
                    insIIRq15, insIIRq31, Q15))
      {return RES_ERROR;}

      // Apply convolution
      if (RES_OK != ApplyConvolution(&general[CHANNEL_0][BUF_BEGIN], 
                    convq15, Q15))
      {return RES_ERROR;}

      // Encode DAC1 to PCM
      if (RES_OK != EncodeData(&general[CHANNEL_0][BUF_BEGIN], 
                    &general[CHANNEL_1][BUF_BEGIN], &dac[BUF_BEGIN], Q15))
      {return RES_ERROR;}

        break;

    case 2: // Buffer full, process second half
      
      // Decode ADC1 from PCM
      if (RES_OK != DecodeData(&adc[BUFFER_SIZE], 
                    &general[CHANNEL_0][BUF_HALF],
                    &general[CHANNEL_1][BUF_HALF], Q15))
      {return RES_ERROR;}

      // Normalize channels
      if (RES_OK != NormalizeChannels(&general[CHANNEL_0][BUF_HALF], 
                                      &normGain[CHANNEL_0], Q15))
      {return RES_ERROR;}

      // Filter all channels
      if (RES_OK != ApplyFilters(&general[CHANNEL_0][BUF_HALF], FIRf32, FIRq15,
                    insFIRf32, insFIRq15, IIRf32, IIRq15, IIRq31, insIIRf32, 
                    insIIRq15, insIIRq31, Q15)) 
      {return RES_ERROR;}

      // Apply convolution
      if (RES_OK != ApplyConvolution(&general[CHANNEL_0][BUF_HALF], 
                    convq15, Q15))
      {return RES_ERROR;}

      // Encode DAC1 to PCM
      if (RES_OK != EncodeData(&general[CHANNEL_0][BUF_HALF], 
                    &general[CHANNEL_1][BUF_HALF], &dac[BUFFER_SIZE], Q15))
      {return RES_ERROR;}
        break;
    
    default:
      appError = ERROR_CODE_14;
      return RES_ERROR;
        break;
  }
  
  //while(timerCount < 1.01 * DMA_INT_PERIOD) ADD BEFORE ENCODING TO TEST LIMIT
  cpuUsage = (float)timerCount * 100.0 / DMA_INT_PERIOD;
  timerCount = 0; // Performance monitor reset timer
  dataReadyFlag = 0;
  return RES_OK;
}

// *****************************************************************************
static tErrorCode CheckConfig(void)
// *****************************************************************************
// Description: Checks if filter config changed and configures filters if true
// Parameters: none
// Returns: error code
// *****************************************************************************
{
  switch (filterConfigFlag)
  {
    case 0:
      break;
  
    case 1:
      if (RES_OK != DSP_UpdateIIRInstances(filterConfig, IIRf32, IIRq15, IIRq31, 
                    insIIRf32, insIIRq15, insIIRq31, normGain))
      {appError = ERROR_CODE_15; return RES_ERROR;}
    
      if (RES_OK != DSP_UpdateFIRInstances(filterConfig, FIRf32, FIRq15, 
                    insFIRf32, insFIRq15))
      {appError = ERROR_CODE_16; return RES_ERROR;}
    
      if (RES_OK != CheckParams())
      {appError = ERROR_CODE_17; return RES_ERROR;}
      filterConfigFlag = 0;
      break;

    default:
      return RES_ERROR;
      break;
  }

  return RES_OK;
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
