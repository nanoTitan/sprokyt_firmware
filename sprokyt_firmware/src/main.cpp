#include <mbed.h>
//#include "ble.h"
//#include "imu.h"
//#include "error.h"
//
///* Private function prototypes -----------------------------------------------*/
//
///* Private variables ---------------------------------------------------------*/
//Serial pc2(USBTX, USBRX);
//
//int main() 
//{
//	if (HAL_Init() != HAL_OK)
//	{
//		Error_Handler(); 
//	}
//	
//	Init_BLE();
//	InitIMU();
//	
//	while (1)
//	{
//		Update_BLE();
//		UpdateIMU();
//	}
//}

DigitalOut g_LED(LED1);
Serial pc(USBTX, USBRX);

int main() 
{
	pc.baud(9600);
	
	for (;;)
	{
		g_LED = 1;
		wait_ms(500);
		g_LED = 0;
		wait_ms(500);
		
		printf("Nothing happening here: %f\n", 654.123129389);
		pc.printf("This doesn't do anything either %d\n", 100);     // Not working for me either
	}
}