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

#include "wm8994.h"

#include "copy_buffer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define HSEM_ID_0 (0U) /* HW semaphore 0*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C4_Init(void);
static void MX_SAI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RECORD_BUFFER_SIZE  4096

int16_t RecordBuffer[RECORD_BUFFER_SIZE];
int16_t PlaybackBuffer[RECORD_BUFFER_SIZE];

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */
	/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
	/* USER CODE END Boot_Mode_Sequence_0 */

	/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		ErrorHandler();
	}
	/* USER CODE END Boot_Mode_Sequence_1 */
	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();
	/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		ErrorHandler();
	}
	/* USER CODE END Boot_Mode_Sequence_2 */

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C4_Init();
	MX_SAI1_Init();
	/* USER CODE BEGIN 2 */

	if ((wm8994_ReadID(AUDIO_I2C_ADDRESS)) != WM8994_ID) {
		return HAL_ERROR;
	}

	wm8994_Reset(AUDIO_I2C_ADDRESS);

	if (wm8994_Init(AUDIO_I2C_ADDRESS, INPUT_DEVICE_INPUT_LINE_1 | OUTPUT_DEVICE_HEADPHONE , 80, SAI_AUDIO_FREQUENCY_44K) != 0) {
		ErrorHandler();
	}

	if (wm8994_Play(AUDIO_I2C_ADDRESS) != 0) {
		ErrorHandler();
	}

	if (HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t*)RecordBuffer, RECORD_BUFFER_SIZE) != HAL_OK) {
		ErrorHandler();
	}

	if (HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)PlaybackBuffer, RECORD_BUFFER_SIZE) != HAL_OK) {
		ErrorHandler();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	uint32_t onOff = 0;
	uint32_t counter = HAL_GetTick() + 1000;
	for (;;) {
		uint32_t t = HAL_GetTick();
		if (t > counter) {
			HAL_GPIO_WritePin (LED1_GPIO_Port, LED1_Pin, onOff++%2 == 0 ?  GPIO_PIN_SET :  GPIO_PIN_RESET);
			counter = t + 1000;
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

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
	/** Macro to configure the PLL clock source
	 */
	__HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 9;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
	RCC_OscInitStruct.PLL.PLLFRACN = 3072;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		ErrorHandler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		ErrorHandler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1|RCC_PERIPHCLK_I2C4;
	PeriphClkInitStruct.PLL2.PLL2M = 4;
	PeriphClkInitStruct.PLL2.PLL2N = 16;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 7168;
	PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL2;
	PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_D3PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		ErrorHandler();
	}
	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
 * @brief I2C4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C4_Init(void)
{

	/* USER CODE BEGIN I2C4_Init 0 */

	/* USER CODE END I2C4_Init 0 */

	/* USER CODE BEGIN I2C4_Init 1 */

	/* USER CODE END I2C4_Init 1 */
	hi2c4.Instance = I2C4;
	hi2c4.Init.Timing = 0x00702787;
	hi2c4.Init.OwnAddress1 = 0;
	hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c4.Init.OwnAddress2 = 0;
	hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c4) != HAL_OK)
	{
		ErrorHandler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		ErrorHandler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
	{
		ErrorHandler();
	}
	/* USER CODE BEGIN I2C4_Init 2 */

	/* USER CODE END I2C4_Init 2 */

}

/**
 * @brief SAI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SAI1_Init(void)
{

	/* USER CODE BEGIN SAI1_Init 0 */

	/* USER CODE END SAI1_Init 0 */

	/* USER CODE BEGIN SAI1_Init 1 */

	/* USER CODE END SAI1_Init 1 */
	hsai_BlockA1.Instance = SAI1_Block_A;
	hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
	hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
	hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
	hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
	hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockA1.Init.PdmInit.Activation = DISABLE;
	hsai_BlockA1.Init.PdmInit.MicPairsNbr = 1;
	hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
	hsai_BlockA1.FrameInit.FrameLength = 64;
	hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
	hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
	hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
	hsai_BlockA1.SlotInit.FirstBitOffset = 0;
	hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
	hsai_BlockA1.SlotInit.SlotNumber = 4;
	hsai_BlockA1.SlotInit.SlotActive = 0x0000000F;
	if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
	{
		ErrorHandler();
	}
	hsai_BlockB1.Instance = SAI1_Block_B;
	hsai_BlockB1.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_BlockB1.Init.DataSize = SAI_DATASIZE_16;
	hsai_BlockB1.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockB1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_1QF;
	hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockB1.Init.PdmInit.Activation = DISABLE;
	hsai_BlockB1.Init.PdmInit.MicPairsNbr = 1;
	hsai_BlockB1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
	hsai_BlockB1.FrameInit.FrameLength = 64;
	hsai_BlockB1.FrameInit.ActiveFrameLength = 32;
	hsai_BlockB1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
	hsai_BlockB1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockB1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
	hsai_BlockB1.SlotInit.FirstBitOffset = 0;
	hsai_BlockB1.SlotInit.SlotSize = SAI_SLOTSIZE_16B;
	hsai_BlockB1.SlotInit.SlotNumber = 4;
	hsai_BlockB1.SlotInit.SlotActive = 0x0000000F;
	if (HAL_SAI_Init(&hsai_BlockB1) != HAL_OK)
	{
		ErrorHandler();
	}
	/* USER CODE BEGIN SAI1_Init 2 */

	/* USER CODE END SAI1_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMAMUX1_OVR_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMAMUX1_OVR_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMAMUX1_OVR_IRQn);

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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOI, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : CEC_CK_MCO1_Pin */
	GPIO_InitStruct.Pin = CEC_CK_MCO1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(CEC_CK_MCO1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
	GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*AnalogSwitch Config */
	HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_OPEN);

}

/* USER CODE BEGIN 4 */

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
	CopyBuffer(&PlaybackBuffer[0], &RecordBuffer[0], RECORD_BUFFER_SIZE / 2);
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
	CopyBuffer(&PlaybackBuffer[RECORD_BUFFER_SIZE / 2], &RecordBuffer[RECORD_BUFFER_SIZE / 2], RECORD_BUFFER_SIZE / 2);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) { }
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai) { }
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
	ErrorHandler();
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void ErrorHandler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	for(;;) {
		int delay = 100;
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay/2);
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
