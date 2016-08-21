/**
  *******************************************************************************
  * @file    Projects/Multi/Applications/DataLogFusion/Src/stm32f4xx_hal_msp.c
  * @author  CL
  * @version V1.5.0
  * @date    4-April-2016
  * @brief   HAL MSP module.
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  * ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cube_hal.h"
#include "wifi_module.h"
#include "wifi_globals.h"
#include  "error.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief RTC MSP Initialization
  *        This function configures the hardware resources used in this example
  * @param hrtc: RTC handle pointer
  *
  * @note  Care must be taken when HAL_RCCEx_PeriphCLKConfig() is used to select
  *        the RTC clock source; in this case the Backup domain will be reset in
  *        order to modify the RTC Clock source, as consequence RTC registers (including
  *        the backup registers) and RCC_BDCR register are set to their reset values.
  *
  * @retval None
  */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;
  
  /*##-1- Configue LSE as RTC clock soucre ###################################*/
#ifdef RTC_CLOCK_SOURCE_LSE
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
#elif defined (RTC_CLOCK_SOURCE_LSI)
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
#else
#error Please select the RTC Clock source inside the main.h file
#endif /*RTC_CLOCK_SOURCE_LSE*/
  
  /*##-2- Enable RTC peripheral Clocks #######################################*/
  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();
  
  /*##-3- Configure the NVIC for RTC TimeStamp ###################################*/
  HAL_NVIC_SetPriority(TAMP_STAMP_IRQn, 0x0F, 0);
  HAL_NVIC_EnableIRQ(TAMP_STAMP_IRQn);
}

/**
  * @brief RTC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  /*##-1- Reset peripherals ##################################################*/
  __HAL_RCC_RTC_DISABLE();
  
  /*##-2- Disable the NVIC for TimeStamp #####################################*/
  HAL_NVIC_DisableIRQ(TAMP_STAMP_IRQn);
}


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock enable */
  TIM_SF_CLK_ENABLE();
  
  /*##-2- Configure the NVIC for TIMx #########################################*/
  /* Set the TIMx priority */
	HAL_NVIC_SetPriority(TIM_SF_IRQn, 10, 0);
  
  /* Enable the TIMx global Interrupt */
	HAL_NVIC_EnableIRQ(TIM_SF_IRQn);
}



/**
  * @brief TIM MSP DeInitialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* TIMx Peripheral clock disable */
  TIM_SF_CLK_DISABLE();
  
  /* Disable the TIMx global Interrupt */
	HAL_NVIC_DisableIRQ(TIM_SF_IRQn);
}

/**
  * @brief UART MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - NVIC configuration for UART interrupt request enable
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{ 
	GPIO_InitTypeDef  GPIO_InitStruct;

	if (huart == &UartWiFiHandle)
	{
	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
		USARTx_TX_GPIO_CLK_ENABLE();
		USARTx_RX_GPIO_CLK_ENABLE();


		  /* Enable USARTx clock */
		USARTx_CLK_ENABLE(); 
		__SYSCFG_CLK_ENABLE();

#ifdef USE_STM32F1xx_NUCLEO      
		__HAL_AFIO_REMAP_USART3_PARTIAL();
#endif
   
		 /*##-2- Configure peripheral GPIO ##########################################*/  
		 /* UART TX GPIO pin configuration  */
		GPIO_InitStruct.Pin       = WiFi_USART_TX_PIN;
		GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO) 
		GPIO_InitStruct.Pull      = GPIO_NOPULL;
		GPIO_InitStruct.Alternate = WiFi_USARTx_TX_AF;
#endif  
#ifdef USE_STM32F1xx_NUCLEO      
		GPIO_InitStruct.Pull     = GPIO_PULLUP;
#endif
		GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  
		HAL_GPIO_Init(WiFi_USART_TX_GPIO_PORT, &GPIO_InitStruct);

		  /* UART RX GPIO pin configuration  */
		GPIO_InitStruct.Pin = WiFi_USART_RX_PIN;
#ifdef USE_STM32F1xx_NUCLEO  
		GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
#endif  
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
		GPIO_InitStruct.Alternate = WiFi_USARTx_RX_AF;
#endif
  
		HAL_GPIO_Init(WiFi_USART_RX_GPIO_PORT, &GPIO_InitStruct);
  
  
		/* UART RTS GPIO pin configuration  */
		GPIO_InitStruct.Pin = WiFi_USART_RTS_PIN;
#ifdef USE_STM32F1xx_NUCLEO  
		GPIO_InitStruct.Pull     = GPIO_PULLDOWN;
		GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;//GPIO_MODE_AF_PP;
#endif  
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
		GPIO_InitStruct.Pull     = GPIO_PULLUP;
		GPIO_InitStruct.Alternate = WiFi_USARTx_RX_AF;
#endif
  
		HAL_GPIO_Init(WiFi_USART_RTS_GPIO_PORT, &GPIO_InitStruct);
  
		/*##-3- Configure the NVIC for UART ########################################*/
		/* NVIC for USART */
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
		HAL_NVIC_SetPriority(USARTx_IRQn, 3, 0);
#else
		HAL_NVIC_SetPriority(USARTx_IRQn, 3, 0);
#endif
		HAL_NVIC_EnableIRQ(USARTx_IRQn);
	}
}


/**
  * @brief UART MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if (huart == &UartWiFiHandle)
	{
	/*##-1- Reset peripherals ##################################################*/
		USARTx_FORCE_RESET();
		USARTx_RELEASE_RESET();

		  /*##-2- Disable peripherals and GPIO Clocks #################################*/
		  /* Configure UART Tx as alternate function  */
		HAL_GPIO_DeInit(WiFi_USART_TX_GPIO_PORT, WiFi_USART_TX_PIN);
		/* Configure UART Rx as alternate function  */
		HAL_GPIO_DeInit(WiFi_USART_RX_GPIO_PORT, WiFi_USART_RX_PIN);
  
		/*##-3- Disable the NVIC for UART ##########################################*/
		HAL_NVIC_DisableIRQ(USARTx_IRQn);
	}
}


/**
  * @brief TIM MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	if (htim == &PushTimHandle)
	{
	  /* TIMx Peripheral clock enable */
		TIM_WIFI_P_CLK_ENABLE();
    
		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM_WIFI_P_IRQn, 3, 0);
    
		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM_WIFI_P_IRQn);
	}
	else
	{    
	  /* TIMx Peripheral clock enable */
		TIM_WIFI_CLK_ENABLE();
    
		/*##-2- Configure the NVIC for TIMx ########################################*/
		/* Set the TIMx priority */
		HAL_NVIC_SetPriority(TIM_WIFI_IRQn, 3, 0);
    
		/* Enable the TIMx global Interrupt */
		HAL_NVIC_EnableIRQ(TIM_WIFI_IRQn);
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
