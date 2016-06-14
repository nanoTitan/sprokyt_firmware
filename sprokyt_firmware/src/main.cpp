#include <mbed.h>
#include "ble.h"

/* Private function prototypes -----------------------------------------------*/
static void Error_Handler(void);

/* Private variables ---------------------------------------------------------*/
Serial pc2(USBTX, USBRX);

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
	while (1)
	{
		pc2.printf("test test!!!");
	}
}