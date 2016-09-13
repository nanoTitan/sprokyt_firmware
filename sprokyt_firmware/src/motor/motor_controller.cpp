#include "motor_controller.h"
#include "mbed.h"
#include "constants.h"
#include "math_ext.h"

/* Private Variables ------------------------------------------------------------------*/
PwmOut _bldcArray[4] = { D9, D10, PC_9, PC_8  };
Timeout _motorArmTimeout;
bool _motorsArmed = false;

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();
static void MotorController_armESCs();

void MotorController_init()
{	
	_bldcArray[0].period_us(20000);				// 20000 us = 50 Hz, 2040.8us = 490 Hz, 83.3 us = 12 Khz	
	_bldcArray[1].period_us(20000);
	_bldcArray[2].period_us(20000);
	_bldcArray[3].period_us(20000);
	
	//MotorController_setMotor(MOTOR_ALL, 2000, 0);
	//wait_ms(3000);
	//MotorController_setMotor(MOTOR_ALL, 1000, 0);
}

void MotorController_armESCs()
{
	for (int i = 1050; i < 2000; i += 50)
	{		
		MotorController_setMotor(MOTOR_ALL, i, 0);
		wait_ms(10);
	}
	
	for (int i = 1950; i >= 1000; i -= 50)
	{
		MotorController_setMotor(MOTOR_ALL, i, 0);
		wait_ms(10);
	}
	
	wait_ms(4000);
}

void MotorController_callibrateESCs()
{
	for (int i = 1050; i <= 2000; i+=50)
	{		
		MotorController_setMotor(MOTOR_D, i, 0);
		wait_ms(10);
	}
	
	MotorController_setMotor(MOTOR_D, 2000, 0);
	wait_ms(4000);
	
	for (int i = 1950; i >= 1000; i-=50)
	{
		MotorController_setMotor(MOTOR_D, i, 0);
		wait_ms(10);
	}
	
	wait_ms(4000);
}

void MotorController_setMotor(uint8_t motorIndxMask, float power, uint8_t direction)
{
	// 1000us - 2000us is 0% - 100% power respectively
//	float x = clampf(power, 0, 1.0f);
	
	if (motorIndxMask & 0x01)
		_bldcArray[0].pulsewidth_us(power);	
	if (motorIndxMask & 0x02)
		_bldcArray[1].pulsewidth_us(power);	
	if (motorIndxMask & 0x04)
		_bldcArray[2].pulsewidth_us(power);	
	if (motorIndxMask & 0x08)
		_bldcArray[3].pulsewidth_us(power);	
}

void MotorController_armMotors()
{
	for (int i = 0; i < MC_NUM_MOTORS; ++i)
	{
		_bldcArray[i].pulsewidth_us(1000);			// 1000us - 2000us equals 0% - 100% power respectively
	}
	
	_motorsArmed = true;
}

bool MotorController_isArmed()
{
	return _motorsArmed;
}