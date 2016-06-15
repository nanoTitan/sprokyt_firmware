#include <mbed.h>
#include "ble.h"
#include "error.h"

/* Private function prototypes -----------------------------------------------*/

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