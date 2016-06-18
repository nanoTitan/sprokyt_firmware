#include <mbed.h>
#include "BLE.h"
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
	
	BLE::InitBLE();
	Init_IMU();
	MotorController::InitMotors();
	
	//Start_IMU();	// Starts the IMU thread
	
	while (1)
	{
		BLE::Update();
		UpdateIMU();
	}
}