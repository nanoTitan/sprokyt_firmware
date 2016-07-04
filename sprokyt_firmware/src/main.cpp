#include <mbed.h>
#include "control_manager.h"
#include "BLE.h"
#include "imu.h"
#include "motor_controller.h"
#include "error.h"

int main()
{
	if (HAL_Init() != HAL_OK)
	{
		Error_Handler(); 
	}
	
	// Must initialize ControlManager before 
	MotorController_init();
	wait_ms(3000);
	
	ControlMgr_init();
	ControlMgr_setType(CONTROLLER_FLIGHT);
	BLE::InitBLE();
	IMU_init();
	
	while (1)
	{		
		BLE::Update();
		IMU_update();
		ControlMgr_update();
	}
}


//
//int main() 
//{
//	if (HAL_Init() != HAL_OK)
//	{
//		Error_Handler(); 
//	}
//	
//	MotorController_init();
//	wait_ms(3000);
//	
//	MotorController_setMotor(0x01, 20, 0);	
//	wait_ms(3000);
//	
//	MotorController_setMotor(0x01, 0, 0);
//	MotorController_setMotor(0x02, 20, 0);
//	wait_ms(3000);
//	
//	MotorController_setMotor(0x02, 0, 0);
//	MotorController_setMotor(0x04, 20, 0);
//	wait_ms(3000);
//	
//	MotorController_setMotor(0x04, 0, 0);
//	MotorController_setMotor(0x08, 20, 0);
//	wait_ms(3000);
//	
//	MotorController_setMotor(0x08, 0, 0);
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