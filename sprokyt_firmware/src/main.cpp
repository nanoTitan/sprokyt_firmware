#include <mbed.h>
#include "stm32f4xx_hal_conf.h"
#include "ble.h"

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/
DigitalOut g_LED(LED1);	// LED5

int main() 
{
	if (HAL_Init() != HAL_OK)
	{
		Error_Handler(); 
	}
	
	Init_BLE();
	
	while (1)
	{
		Update_BLE();
	}
}

static void Error_Handler(void)
{
  /* Turn LED5 on */
	g_LED = 1;
	while (1)
	{
	}
}