#include "LEDManager.h"
#include "stm32f4xx_nucleo.h"

void LEDMgr_Init()
{
	// System board LEDs
	BSP_LED_Init(LED_2);
	BSP_LED_Off(LED_2);
	
	// Case LEDs
	GPIO_InitTypeDef  GPIO_InitStruct;
  
	/* Enable the GPIO_LED Clock */
	__GPIOB_CLK_ENABLE();
  
	/* Configure the GPIO_LED 15 pin */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); 
	
	/* Configure the GPIO_LED 14 pin */
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
}