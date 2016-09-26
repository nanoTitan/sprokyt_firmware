//#include <mbed.h>
#include "control_manager.h"
//#include "BLE.h"
//#include "SWPF01SA.h"
#include "Wifi.h"
#include "imu.h"
#include "motor_controller.h"
#include "error.h"
#include "debug.h"


#ifdef __cplusplus
extern "C" {
#endif 
	
#include "cube_hal.h"

#ifdef __cplusplus
}
#endif

int main()
{	
	PRINTF("Ruka Firmware version 0.1\r\n");
	
	if (HAL_Init() != HAL_OK)
	{
		CError_Handler(); 
	}
	
	// Configure the system clock
	SystemClock_Config();
	
	BSP_LED_Init(LED_2);
	BSP_LED_Off(LED_2);
	
	// Initialize Button
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	
	// Must initialize ControlManager before 
	//MotorController_init();	
	//ControlMgr_init();
	//ControlMgr_setType(CONTROLLER_FLIGHT);	//  CONTROLLER_ESC_PROGRAMMER CONTROLLER_FLIGHT
	//BLE::InitBLE();
	//SWPF01SA::Instance()->InitWifi();
	//Wifi::Instance()->Init();
	IMU_init();
	
	
	while (1)
	{
		//BLE::Update();
		//SWPF01SA::Instance()->Update();
		//Wifi::Instance()->Update();
		IMU_update();
		//ControlMgr_update();
	}
}


//int main() 
//{
//	if (HAL_Init() != HAL_OK)
//	{
//		Error_Handler(); 
//	}
//	
//	MotorController_init();
//	
////	MotorController_setMotor(MOTOR_A, 1200, 0);	
////	wait_ms(3000);
//	MotorController_setMotor(MOTOR_ALL, 1200, 0);
//	wait_ms(3000);
//	
//	MotorController_setMotor(MOTOR_ALL, 1000, 0);
//	
//	while (1)
//	{
//	}
//}