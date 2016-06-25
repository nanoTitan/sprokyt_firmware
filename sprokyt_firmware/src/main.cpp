#include <mbed.h>
#include "ControlManager.h"
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
	
	// Must initialize ControlManager before 
	ControlManager::Instance()->CreateController(CONTROLLER_FLIGHT);
	BLE::InitBLE();
	IMU::Instance()->InitIMU();
	MotorController::InitMotors();
	
	while (1)
	{		
		BLE::Update();
		IMU::Instance()->UpdateIMU();
		ControlManager::Instance()->Update();
	}
}



//int main() 
//{
//	if (HAL_Init() != HAL_OK)
//	{
//		Error_Handler(); 
//	}
//	
//	MotorController::InitMotors();
//	wait_ms(3000);
//	
//	MotorController::SetMotor(0x0F, 20, 0);
//	
//	while (1)
//	{		
//		/*
//		for (uint8_t i = 25; i <= 255; i += 25)
//		{
//			MotorController::SetMotor(0x06, i, 0);
//			wait_ms(500);
//		}
//		*/
//	}
//	
//	MotorController::SetMotor(0x06, 0, 0);
//}