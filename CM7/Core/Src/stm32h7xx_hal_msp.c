/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32h7xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */
 
/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief I2C MSP Initialization
* This function configures the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspInit 0 */

  /* USER CODE END I2C4_MspInit 0 */
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**I2C4 GPIO Configuration    
    PD12     ------> I2C4_SCL
    PD13     ------> I2C4_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C4_CLK_ENABLE();
  /* USER CODE BEGIN I2C4_MspInit 1 */

  /* USER CODE END I2C4_MspInit 1 */
  }

}

/**
* @brief I2C MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hi2c: I2C handle pointer
* @retval None
*/
void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
  if(hi2c->Instance==I2C4)
  {
  /* USER CODE BEGIN I2C4_MspDeInit 0 */

  /* USER CODE END I2C4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C4_CLK_DISABLE();
  
    /**I2C4 GPIO Configuration    
    PD12     ------> I2C4_SCL
    PD13     ------> I2C4_SDA 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12|GPIO_PIN_13);

  /* USER CODE BEGIN I2C4_MspDeInit 1 */

  /* USER CODE END I2C4_MspDeInit 1 */
  }

}

extern DMA_HandleTypeDef hdma_sai1_a;

extern DMA_HandleTypeDef hdma_sai1_b;

static uint32_t SAI1_client =0;

void HAL_SAI_MspInit(SAI_HandleTypeDef* hsai)
{

  GPIO_InitTypeDef GPIO_InitStruct;
/* SAI1 */
    if(hsai->Instance==SAI1_Block_A)
    {
    /* Peripheral clock enable */
    if (SAI1_client == 0)
    {
       __HAL_RCC_SAI1_CLK_ENABLE();
    }
    SAI1_client ++;
    
    /**SAI1_A_Block_A GPIO Configuration    
    PE5     ------> SAI1_SCK_A
    PE4     ------> SAI1_FS_A
    PE6     ------> SAI1_SD_A
    PG7     ------> SAI1_MCLK_A
    PF7     ------> SAI1_MCLK_B
    PF8     ------> SAI1_SCK_B
    PF9     ------> SAI1_FS_B 
    */
    GPIO_InitStruct.Pin = SAI1_SCK_A_Pin|SAI1_FS_A_Pin|SAI1_SD_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SAI1_MCLK_A_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(SAI1_MCLK_A_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    
    hdma_sai1_a.Instance = DMA1_Stream0;
    hdma_sai1_a.Init.Request = DMA_REQUEST_SAI1_A;
    hdma_sai1_a.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_sai1_a.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_a.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_a.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai1_a.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_a.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_a.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_sai1_a.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai1_a.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_sai1_a.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_sai1_a) != HAL_OK)
    {
      ErrorHandler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_a);

    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_a);

    }
    if(hsai->Instance==SAI1_Block_B)
    {
      /* Peripheral clock enable */
      if (SAI1_client == 0)
      {
       __HAL_RCC_SAI1_CLK_ENABLE();
      }
    SAI1_client ++;
    
    /**SAI1_B_Block_B GPIO Configuration    
    PE3     ------> SAI1_SD_B 
    */
    GPIO_InitStruct.Pin = SAI1_SD_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_SAI1;
    HAL_GPIO_Init(SAI1_SD_B_GPIO_Port, &GPIO_InitStruct);

    /* Peripheral DMA init*/
    
    hdma_sai1_b.Instance = DMA1_Stream1;
    hdma_sai1_b.Init.Request = DMA_REQUEST_SAI1_B;
    hdma_sai1_b.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sai1_b.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sai1_b.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sai1_b.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sai1_b.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sai1_b.Init.Mode = DMA_CIRCULAR;
    hdma_sai1_b.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_sai1_b.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_sai1_b.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_sai1_b.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_sai1_b.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_sai1_b) != HAL_OK)
    {
      ErrorHandler();
    }

    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(hsai,hdmarx,hdma_sai1_b);
    __HAL_LINKDMA(hsai,hdmatx,hdma_sai1_b);
    }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef* hsai)
{
/* SAI1 */
    if(hsai->Instance==SAI1_Block_A)
    {
    SAI1_client --;
    if (SAI1_client == 0)
      {
      /* Peripheral clock disable */ 
       __HAL_RCC_SAI1_CLK_DISABLE();
      }
    
    /**SAI1_A_Block_A GPIO Configuration    
    PE5     ------> SAI1_SCK_A
    PE4     ------> SAI1_FS_A
    PE6     ------> SAI1_SD_A
    PG7     ------> SAI1_MCLK_A
    PF7     ------> SAI1_MCLK_B
    PF8     ------> SAI1_SCK_B
    PF9     ------> SAI1_FS_B 
    */
    HAL_GPIO_DeInit(GPIOE, SAI1_SCK_A_Pin|SAI1_FS_A_Pin|SAI1_SD_A_Pin);

    HAL_GPIO_DeInit(SAI1_MCLK_A_GPIO_Port, SAI1_MCLK_A_Pin);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);

    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
    if(hsai->Instance==SAI1_Block_B)
    {
    SAI1_client --;
      if (SAI1_client == 0)
      {
      /* Peripheral clock disable */
      __HAL_RCC_SAI1_CLK_DISABLE();
      }
    
    /**SAI1_B_Block_B GPIO Configuration    
    PE3     ------> SAI1_SD_B 
    */
    HAL_GPIO_DeInit(SAI1_SD_B_GPIO_Port, SAI1_SD_B_Pin);

    HAL_DMA_DeInit(hsai->hdmarx);
    HAL_DMA_DeInit(hsai->hdmatx);
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
