#include <mbed.h>
#include "ble.h"
#include "imu.h"
#include "MotorController.h"
#include "error.h"
#include "test.h"

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
	MotorController::InitMotors();
	
	//Start_IMU();	// Starts the IMU thread
	
	while (1)
	{
		Update_BLE();
		UpdateIMU();
	}
}