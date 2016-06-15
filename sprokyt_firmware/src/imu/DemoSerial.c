/**
  ******************************************************************************
  * @file    Projects/Multi/Applications/DataLogFusion/Src/DemoSerial.c
  * @author  CL
  * @version V1.5.0
  * @date    4-April-2016
  * @brief   Handler AST Serial Protocol
  ******************************************************************************
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
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "DemoSerial.h"
#include "MotionFX_Manager.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "main.h"
#include "imu.h"
#include "com.h"
#include "error.h"

/** @addtogroup OSX_MOTION_FX_Applications
  * @{
  */

/** @addtogroup DATALOGFUSION
  * @{
  */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint32_t Sensors_Enabled;
extern volatile uint32_t DataTxPeriod;
extern volatile uint8_t SF_Active;
extern volatile uint8_t SF_change;
extern volatile uint8_t SF_6x_enabled;
extern TIM_HandleTypeDef DataLogTimHandle;
extern void *ACCELERO_handle;
extern void *GYRO_handle;
extern void *MAGNETO_handle;
extern void *HUMIDITY_handle;
extern void *TEMPERATURE_handle;
extern void *PRESSURE_handle;

/* Private variables ---------------------------------------------------------*/
volatile uint8_t DataLoggerActive;
volatile uint8_t SenderInterface = 0;
uint8_t PresentationString[] = {"MEMS shield demo"};
volatile uint8_t DataStreamingDest = 1;
uint16_t timer_period = 1000 - 1;
uint16_t timer_pulse = (1000) / 2;

/* Private Functions ----------------------------------------------------------*/

/**
 * @brief  Initialize DataLog timer
 * @param  None
 * @retval None
 */
void DataLogTimerInit(void)
{
	TIM_OC_InitTypeDef sConfig;
	uint16_t uhPrescalerValue;
	//uint16_t uhCapturedValue = 0;
  
	/* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	uhPrescalerValue = (uint16_t)((SystemCoreClock) / DATALOG_TIM_COUNTER_CLK) - 1;
  
	/*##-1- Configure the TIM peripheral #######################################*/
	/* Set TIMx instance */
	DataLogTimHandle.Instance = TIMDataLog;
  
	/* Initialize TIM3 peripheral as follow:
	+ Period = 65535
	+ Prescaler = (SystemCoreClock/2)/60000
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
  
	timer_period = (DATALOG_TIM_COUNTER_CLK / (1000 / DATALOG_TIM_PERIOD)) - 1;
	timer_pulse = timer_period / 2;
	
	DataLogTimHandle.Init.Period = timer_period;
	DataLogTimHandle.Init.Prescaler = uhPrescalerValue;
	DataLogTimHandle.Init.ClockDivision = 0;
	DataLogTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	DataLogTimHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_OC_Init(&DataLogTimHandle) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}
  
	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration */
	sConfig.OCMode = TIM_OCMODE_TIMING;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;
  
	/* Set the pulse value for channel 1 */
	sConfig.Pulse = timer_pulse;
  
	if (HAL_TIM_OC_ConfigChannel(&DataLogTimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}
  
  
	/*##-4- Start the Output Compare mode in interrupt mode ####################*/
	/* Start Channel1 */
	if (HAL_TIM_OC_Start_IT(&DataLogTimHandle, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}
  
}

/**
  * @brief  Build the reply header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_REPLY_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] += CMD_Reply_Add;
}

/**
  * @brief  Build the nack header
  * @param  Msg the pointer to the message to be built
  * @retval None
  */
void BUILD_NACK_HEADER(TMsg *Msg)
{
  Msg->Data[0] = Msg->Data[1];
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_NACK;
}

/**
  * @brief  Initialize the streaming header
  * @param  Msg the pointer to the header to be initialized
  * @retval None
  */
void INIT_STREAMING_HEADER(TMsg *Msg)
{
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  Msg->Len = 3;
}

/**
  * @brief  Initialize the streaming message
  * @param  Msg the pointer to the message to be initialized
  * @retval None
  */
void INIT_STREAMING_MSG(TMsg *Msg)
{
  uint8_t i;
  
  Msg->Data[0] = DataStreamingDest;
  Msg->Data[1] = DEV_ADDR;
  Msg->Data[2] = CMD_Start_Data_Streaming;
  for(i = 3; i < STREAMING_MSG_LENGTH + 3; i++)
  {
    Msg->Data[i] = 0;
  }
  Msg->Len = 3;
  
}

/**
  * @brief  Handle a message
  * @param  Msg the pointer to the message to be handled
  * @retval 1 if the message is correctly handled, 0 otherwise
  */
int HandleMSG(TMsg *Msg)
//  DestAddr | SouceAddr | CMD | PAYLOAD
//      1          1        1       N
{
  uint32_t i;
  uint8_t instance;
  
  if (Msg->Len < 2) return 0;
  if (Msg->Data[0] != DEV_ADDR) return 0;
  switch (Msg->Data[2])   // CMD
  {
  
    case CMD_Ping:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_Enter_DFU_Mode:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      return 1;
      
    case CMD_Read_PresString:
      if (Msg->Len != 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      i = 0; //
      while (i < (sizeof(PresentationString) - 1))
      {
        Msg->Data[3 + i] = PresentationString[i];
        i++;
      }
      
      Msg->Len = 3 + i;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_CheckModeSupport:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Serialize_s32(&Msg->Data[3], DATALOG_FUSION_MODE, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_LPS25HB_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
        
      BSP_PRESSURE_Get_Instance( PRESSURE_handle, &instance );
        
      switch (instance)
      {
        case LPS25HB_P_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;
        case LPS25HB_P_1:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;
        case LPS22HB_P_0:
          Serialize_s32(&Msg->Data[3], 3, 4);
          Msg->Len = 3 + 4;
          break;
        default:
          break;
      }
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_HTS221_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg ); 
      Serialize_s32(&Msg->Data[3], 1, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_LSM6DSO_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );
      
      /* We can check one between accelerometer instance and gyroscope instance */
      BSP_GYRO_Get_Instance( GYRO_handle, &instance );
          
      switch (instance)
      {
        case LSM6DS0_G_0:
          Serialize_s32(&Msg->Data[3], 1, 4);
          Msg->Len = 3 + 4;
          break;
        case LSM6DS3_G_0:
          Serialize_s32(&Msg->Data[3], 2, 4);
          Msg->Len = 3 + 4;
          break;
        default:
          break;
      }

      UART_SendMsg(Msg);
      return 1;
      
    case CMD_LIS3MDL_Init:
      if (Msg->Len < 3) return 0;

      BUILD_REPLY_HEADER( Msg );  
      Serialize_s32(&Msg->Data[3], 1, 4);
      Msg->Len = 3 + 4;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_Start_Data_Streaming:
      if (Msg->Len < 3) return 0;
      Sensors_Enabled = Deserialize(&Msg->Data[3], 4);
      DataTxPeriod = Deserialize(&Msg->Data[7], 4);
      DataLoggerActive = 1;
      DataStreamingDest = Msg->Data[1];
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_Stop_Data_Streaming:
      if (Msg->Len < 3) return 0;
      Sensors_Enabled = 0;
      DataLoggerActive = 0;
      SF_Active = 0;
      HAL_TIM_OC_Stop(&DataLogTimHandle, TIM_CHANNEL_1);
      
      if(SF_6x_enabled == 1)
      {
        MotionFX_manager_stop_6X();
        MotionFX_manager_start_9X();
        SF_6x_enabled = 0;
      }
      
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      return 1;
    case CMD_ChangeSF:
      if (Msg->Len < 3) return 0;
      
      SF_change = 1;
      
      BUILD_REPLY_HEADER(Msg);
      UART_SendMsg(Msg);
      return 1;
    case CMD_Set_DateTime:
      if (Msg->Len < 3) return 0;
      BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      RTC_TimeRegulate(Msg->Data[3], Msg->Data[4], Msg->Data[5]);
      UART_SendMsg(Msg);
      return 1;
      
    case CMD_SF_Init:
      if (Msg->Len < 3) return 0;
      DataStreamingDest = Msg->Data[1];
      DataLogTimerInit();
	  BUILD_REPLY_HEADER(Msg);
      Msg->Len = 3;
      UART_SendMsg(Msg);
      SF_Active = 1;
      return 1;
      
    default:
      return 0;
  }
}

/**
 * @}
 */

/**
 * @}
 */
