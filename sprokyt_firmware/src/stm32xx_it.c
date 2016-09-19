/**
  ******************************************************************************
  * @file    stm32xx_it.c 
  * @author  CL
  * @version V1.0.0
  * @date    04-July-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "stm32xx_it.h"
#include "wifi_module.h"
#include "stm32_spwf_wifi.h" 
#include "wifi_globals.h"
#include "debug.h"

/** @addtogroup X-CUBE-BLE1_Applications
 *  @{
 */

/** @addtogroup SensorDemo
 *  @{
 */
 
/** @defgroup INTERRUPT_HANDLER 
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t ms_counter = 0;
volatile uint8_t button_event = 0;
extern TIM_HandleTypeDef SFTimHandle;
extern SPI_HandleTypeDef SpiHandle;			/* SPI handler declared in "main.c" file */

/* Private function prototypes -----------------------------------------------*/
void USART1_IRQHandler(void);    
void USART1_PRINT_IRQHandler(void);
void USART1_EXTI_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM2_IRQHandler(void);

/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief  NMI_Handler This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

void prvGetRegistersFromStack(uint32_t *pulFaultStackAddress)
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

	r0 = pulFaultStackAddress[0];
	r1 = pulFaultStackAddress[1];
	r2 = pulFaultStackAddress[2];
	r3 = pulFaultStackAddress[3];

	r12 = pulFaultStackAddress[4];
	lr = pulFaultStackAddress[5];
	pc = pulFaultStackAddress[6];
	psr = pulFaultStackAddress[7];
	
	PRINTF("\n\n[Hard fault handler - all numbers in hex]\n");
	PRINTF("R0 = %x\r\n", r0);
	PRINTF("R1 = %x\r\n", r1);
	PRINTF("R2 = %x\r\n", r2);
	PRINTF("R3 = %x\r\n", r3);
	PRINTF("R12 = %xv\n", r12);
	PRINTF("LR [R14] = %x  subroutine call return addressv\n", lr);
	PRINTF("PC [R15] = %x  program counter\r\n", pc);
	PRINTF("PSR = %x\r\n", psr);
	PRINTF("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
	PRINTF("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
	PRINTF("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
	PRINTF("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
	PRINTF("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
	PRINTF("SCB_SHCSR = %x\r\n", SCB->SHCSR);

	    /* When the following line is hit, the variables contain the register values. */
	for (;;);
}

/**
* @brief  HardFault_Handler This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
	PRINTF("Hard Fault Exception\r\n");
	
	/* Go to infinite loop when Hard Fault exception occurs */
	__asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
	PRINTF("Memory Manage Exception\r\n");
	
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{ 
	}
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
	PRINTF("Bus Fault Exception\r\n");
	
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
* @brief  SVC_Handler This function handles SVCall exception.
* @param  None
* @retval None
*/
/*
void SVC_Handler(void)
{
	PRINTF("SVC Fault Exception\r\n");
	
	while (1)
	{
	}
}
*/

/**
* @brief  DebugMon_Handler This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
	PRINTF("Debug Monitor Exception\r\n");
	
	while (1)
	{
	}
}

/**
* @brief  PendSV_Handler This function handles PendSVC exception.
* @param  None
* @retval None
*/
/*
void PendSV_Handler(void)
{
	PRINTF("PendSVC Exception\r\n");
	
	while (1)
	{
	}
}
*/

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/

/**
* @brief  SysTick_Handler This function handles SysTick Handler.
* @param  None
* @retval None
*/
/*
void SysTick_Handler(void)
{
	HAL_IncTick();
	Wifi_SysTick_Isr();
	ms_counter++;
}
*/

/**
  * @brief  This function GPIO EXTI Callback.
  * @param  Pin number of the GPIO generating the EXTI IRQ
  * @retval None
  */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
  //RX_EXTI_Isr(GPIO_Pin);
//}

/**
* @brief  This function handles External line interrupt request for BlueNRG.
* @param  None
* @retval None
*/
void BNRG_SPI_EXTI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(BNRG_SPI_EXTI_PIN);
}


/**
* @brief  This function handles the Push Button interrupt request.
* @param  None
* @retval None
*/
void PUSH_BUTTON_EXTI_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
  
  button_event = 1;
}

/**
* @brief  This function handles TIM interrupt request.
* @param  None
* @retval None
*/
void TIM_SF_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&SFTimHandle);
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM_WIFI_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&TimHandle);
  
}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIM_WIFI_P_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&PushTimHandle);
  
}

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*
void PPP_IRQHandler(void)
{
}
*/

/**
* @brief  Period elapsed callback in non blocking mode
*         This timer is used for calling back User registered functions with information
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
	Wifi_TIM_Handler(htim);
}

/**
* @brief  HAL_UART_RxCpltCallback
*         Rx Transfer completed callback
* @param  UsartHandle: UART handle 
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
	WiFi_HAL_UART_RxCpltCallback(UartHandleArg);
}

/**
* @brief  HAL_UART_TxCpltCallback
*         Tx Transfer completed callback
* @param  UsartHandle: UART handle 
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
	WiFi_HAL_UART_TxCpltCallback(UartHandleArg);
}

/**
  * @brief  UART error callbacks
  * @param  UsartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	WiFi_HAL_UART_ErrorCallback(UartHandle);
}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers						  */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).												  */
/******************************************************************************/

/**
  * @brief  This function handles USARTx Handler.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UartWiFiHandle);
}

/**
  * @brief  This function handles USARTx vcom Handler.
  * @param  None
  * @retval None
  */
#ifdef USART_PRINT_MSG
void USARTx_PRINT_IRQHandler(void)
{
	HAL_UART_IRQHandler(&UartMsgHandle);
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
