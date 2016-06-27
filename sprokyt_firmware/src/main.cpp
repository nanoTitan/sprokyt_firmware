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
	MotorController::InitMotors();
	wait_ms(3000);
	
	ControlManager::Instance()->CreateController(CONTROLLER_FLIGHT);
	BLE::InitBLE();
	IMU::Instance()->InitIMU();
	
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
//	MotorController::SetMotor(0x01, 20, 0);	
//	wait_ms(3000);
//	
//	MotorController::SetMotor(0x01, 0, 0);
//	MotorController::SetMotor(0x02, 20, 0);
//	wait_ms(3000);
//	
//	MotorController::SetMotor(0x02, 0, 0);
//	MotorController::SetMotor(0x04, 20, 0);
//	wait_ms(3000);
//	
//	MotorController::SetMotor(0x04, 0, 0);
//	MotorController::SetMotor(0x08, 20, 0);
//	wait_ms(3000);
//	
//	MotorController::SetMotor(0x08, 20, 0);
//	wait_ms(5000);
//	MotorController::SetMotor(0x08, 0, 0);
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
//}