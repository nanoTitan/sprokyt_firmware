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
	PRINTF("***************************\r\n");
	PRINTF("Ruka Firmware Version %.02f\r\n", FIRMWARE_VERSION);
	PRINTF("Copyright Sprokyt LLC 2016\r\n");
	PRINTF("All Rights Reserved\r\n");
	PRINTF("***************************\r\n\r\n");
	
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
	
	// Motor Controller
	MotorController_init();	
	
	// Control Manager
	ControlMgr_init();
	ControlMgr_setType(CONTROLLER_FLIGHT);	//  CONTROLLER_ESC_PROGRAMMER CONTROLLER_FLIGHT
	
	// Communication
	//BLE::InitBLE();
	//SWPF01SA::Instance()->InitWifi();		// ST Wifi
	Wifi::Instance()->Init();				// ESP Wifi
	
	// IMU and Sensors
	IMU_init();
	
	MotorController_setMotor(MOTOR_A, 1500, DIR_CW);
	
	while (1)
	{
		// Communication
		Wifi::Instance()->Update();
		//BLE::Update();
		//SWPF01SA::Instance()->Update();
		
		// IMU and Sensors
		IMU_update();
		
		ControlMgr_update();
	}
}
