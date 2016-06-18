#include <mbed.h>
#include "ble.h"
#include "imu.h"
#include "motor_controller.h"
#include "error.h"

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

int main() 
{
	if (HAL_Init() != HAL_OK)
	{
		Error_Handler(); 
	}
	
	Init_BLE();
	Init_IMU();
	Init_MotorController();
	
	//Start_IMU();	// Starts the IMU thread
	
	while (1)
	{
		Update_BLE();
		UpdateIMU();
	}
}