#include "rover_control.h"
#include "control_manager.h"
#include "motor_controller.h"
#include "PID.h"
//#include "BLE.h"
#include "Wifi.h"
#include "math_ext.h"
#include "debug.h"
#include <mbed.h>

extern "C" 
{
#include "imu.h"
}

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnected();
static void UpdateDisconnected();
static void Disarm();
static void RunMotorTest();
static void PrintIMU();
static void ParseTranslate(uint8_t _x, uint8_t _y);

/* Private functions ---------------------------------------------------------*/
void RoverControl_init()
{	
}

void RoverControl_update()
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
		UpdateConnected();
		break;
		
	case CONTROL_STATE_DISCONNECTED:
		UpdateDisconnected();
		break;
		
	default:
		break;
	}
}

void UpdateConnected()
{
	//RunMotorTest();
	PrintIMU();
}

void UpdateDisconnected()
{
	// TODO: Show flashing LEDs if connection is lost
	
	Disarm();
}

void Disarm()
{	
	// Turn motors off
	MotorController_setMotor(MOTOR_ALL, MIN_THROTTLE, FWD);
	ControlMgr_setState(CONTROL_STATE_IDLE);
}

void RoverControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	 if (data_length == 0)
		return;
	
	uint8_t instruction = att_data[0];
	if (instruction == INSTRUCTION_TRANSLATE)
	{
		uint8_t _x = att_data[1];
		uint8_t _y = att_data[2];
		ParseTranslate(_x, _y);
	}
}

void ParseTranslate(uint8_t _x, uint8_t _y)
{
	/*
		A - B
		|   |
		D - C
		*/
		
	direction_t dir = FWD;
	float x = mapf(_x, 0, 255, -1, 1);
	float y = mapf(_y, 0, 255, -1, 1);
		
	float e = x;
	if (abs(y) > abs(x))
		e = y;
		
	if (e < 0)
		e = -e;
		
	if (x > 0)
	{
		if (y > 0)
		{		
			float d = y - x;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, d, dir);
			MotorController_setMotor(MOTOR_C, d, dir);
			MotorController_setMotor(MOTOR_B, e, FWD);
			MotorController_setMotor(MOTOR_D, e, FWD);
		}
		else
		{
			float d = x + y;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, e, BWD);
			MotorController_setMotor(MOTOR_C, e, BWD);
			MotorController_setMotor(MOTOR_B, d, dir);
			MotorController_setMotor(MOTOR_D, d, dir);
		}
	}
	else
	{
		if (y > 0)
		{
			float d = x + y;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, e, FWD);
			MotorController_setMotor(MOTOR_C, e, FWD);
			MotorController_setMotor(MOTOR_B, d, dir);
			MotorController_setMotor(MOTOR_D, d, dir);
		}
		else
		{
			float d = y - x;
			if (d < 0)
			{
				dir = BWD;
				d = -d;
			}	
				
			MotorController_setMotor(MOTOR_A, d, dir);
			MotorController_setMotor(MOTOR_C, d, dir);
			MotorController_setMotor(MOTOR_B, e, BWD);
			MotorController_setMotor(MOTOR_D, e, BWD);
		}
	}
}

void RunMotorTest()
{
		// ********* Test code for servos *********
	static float speed = 0;	
	static bool up = true;
	static int motor = MOTOR_ALL;
	
	if (up)
	{
		wait_ms(50);
		MotorController_setMotor(motor, speed, FWD);	// TEST Servos
		speed += 0.01;
		
		if (speed >= 1)
		{
			up = false;
		}	
	}
	else
	{
		wait_ms(50);
		MotorController_setMotor(motor, speed, BWD);	// TEST Servos
		speed -= 0.01;
		
		if (speed <= 0 && motor < MOTOR_D)
		{
			MotorController_setMotor(motor, 0, FWD);	// Stop
			motor <<= 1;
			speed = 0;
			up = true;
		}
	}
	// ********* End Test code for servos ********* 
}

void PrintIMU()
{
	float yaw =  IMU_get_sf_yaw();
	float pitch = IMU_get_sf_pitch();
	float roll = IMU_get_sf_roll();
	
	PRINTF("yaw: %.2f, pitch: %.2f, roll: %.2f\r\n", yaw, pitch, roll);			// yaw, pitch, roll
}