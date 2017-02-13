/*
Ruka Firmware
Copyright Sprokyt LLC 2016
All Rights Reserved
*/

#include "constants.h"
#include "control_manager.h"
//#include "BLE.h"
//#include "SWPF01SA.h"
#include "Wifi.h"
#include "motor_controller.h"
#include "error.h"
#include "debug.h"


#ifdef __cplusplus
extern "C" {
#endif 

#include "cube_hal.h"
#include "imu.h"
#include "LED/LEDManager.h"

#ifdef __cplusplus
}
#endif

int main()
{	
	PRINTF("***************************\r\n");
	PRINTF("Ruka Firmware Version %s\r\n", FIRMWARE_VERSION_STR);
	PRINTF("Copyright Sprokyt LLC 2016\r\n");
	PRINTF("All Rights Reserved\r\n");
	PRINTF("***************************\r\n\r\n");
	
	if (HAL_Init() != HAL_OK)
	{
		CError_Handler(); 
	}
	
	// Configure the system clock
	SystemClock_Config();
	
	LEDMgr_Init();
	
	// Initialize Button
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	
	// Motor Controller
	//MotorController_init();	
	
	// Control Manager
	ControlMgr_init();
	ControlMgr_setType(CONTROLLER_USER);	//  CONTROLLER_ESC_PROGRAMMER CONTROLLER_FLIGHT
	
	// IMU and Sensors
	//IMU_init();
	
	// Communication
	//BLE::InitBLE();
	//SWPF01SA::Instance()->InitWifi();		// ST Wifi
	//Wifi::Instance()->Init();				// ESP Wifi
	
	while (1)
	{
		HAL_Delay(500);
		
		// Communication
		//Wifi::Instance()->Update();
		//BLE::Update();
		//SWPF01SA::Instance()->Update();
		
		// IMU and Sensors
		//IMU_update();
		
		ControlMgr_update();		
	}
}
