#include <mbed.h>
#include "BLE.h"
#include "IMU.h"
#include "MotorController.h"
#include "error.h"

int main() 
{
	if (HAL_Init() != HAL_OK)
	{
		Error_Handler(); 
	}
	
	BLE::InitBLE();
	IMU::Instance()->InitIMU();
	MotorController::InitMotors();
	
	while (1)
	{		
		BLE::Update();
		IMU::Instance()->UpdateIMU();
	}
}


/*
int main() 
{
	if (HAL_Init() != HAL_OK)
	{
		Error_Handler(); 
	}
	
	MotorController::InitMotors();
	wait_ms(3000);
	
	while (1)
	{		
		for (float i = 0.1f; i < 1.0f; i += 0.1f)
		{
			MotorController::SetMotor(0x08, i);
			wait_ms(500);
		}
	}
}
*/