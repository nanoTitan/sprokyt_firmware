#include "user_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "PID.h"
//#include "BLE.h"
#include "Wifi.h"
#include "math_ext.h"
#include "debug.h"
#include <mbed.h>


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void UpdateUserControl();
static void UpdateDisconnected();
static void Disarm();

/* Private functions ---------------------------------------------------------*/
void UserControl_init()
{	
}

void UserControl_update()
{	
	CONTROL_STATE state = ControlMgr_getState();
	
// Debug flight control without begin connected
#if defined(DEBUG_FLIGHT_CONTROL_NO_CONNECT)
	state = CONTROL_STATE_CONNECTED;
#endif 
	
	switch (state)
	{
	case CONTROL_STATE_IDLE:
		break;
		
	case CONTROL_STATE_CONNECTED:
		UpdateUserControl();
		break;
		
	case CONTROL_STATE_DISCONNECTED:
		UpdateDisconnected();
		break;
		
	default:
		break;
	}
}

void UpdateUserControl()
{	
}

void UpdateDisconnected()
{
	// TODO: Show flashing LEDs if connection is lost
	
	// TODO: Measure barometer to know when to exit descent
	
	Disarm();
}

void Disarm()
{	
	// Turn motors off
	MotorController_setMotor(MOTOR_ALL, MIN_THROTTLE, FWD);
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void UserControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	if (data_length == 3)
	{
		uint8_t instruction = att_data[0];
		direction_t dir = FWD;
		if( instruction == INSTRUCTION_FORWARD )
			dir = BWD;
		
		uint8_t motorIndex = att_data[1];
		uint8_t value = att_data[2];
		
		float newValue = map(value, 0, 255, 0, 1);
		//UserControl_setMotor(motorIndex, newValue, dir);
		
		PRINTF("%c, %.4f\r\n", motorIndex, newValue);
	}
}

void UserControl_setMotor(uint8_t motorIndex, float value, direction_t dir)
{
	MotorController_setMotor(motorIndex, value, dir);
}
