#include "MotorController.h"
#include "mbed.h"
#include "math_ext.h"

/* Private Variables ------------------------------------------------------------------*/
PwmOut _bldcArray[4] = { D3, D5, D6, D9 };
Timeout _motorArmTimeout;
bool _motorsArmed = false;

/* Private Functions ------------------------------------------------------------------*/


void MotorController::InitMotors()
{	
	for (int i = 0; i < MC_NUM_MOTORS; ++i)
	{
		_bldcArray[i].period_us(20000);				// 50 hertz
		_bldcArray[i].pulsewidth_us(1000);						// 1000us - 2000us is 0% - 100% power respectively
	}
	
	ArmMotors();
}

void MotorController::SetMotor(uint8_t motorIndxMask, uint8_t power, uint8_t direction)
{
	// 1000us - 2000us is 0% - 100% power respectively
	float x = map(power, 0, 255, 0.0f, 1.0f);
	
	if (motorIndxMask & 0x01)
		_bldcArray[0].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x02)
		_bldcArray[1].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x04)
		_bldcArray[2].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x08)
		_bldcArray[3].pulsewidth_us(1000 + (1000 * x));	
}

void MotorController::ArmMotors()
{
	for (int i = 0; i < MC_NUM_MOTORS; ++i)
	{
		_bldcArray[i].pulsewidth_us(1000);		// 1000us arms motors
	}
	
	_motorArmTimeout.attach(ArmMotorsCallback, 3.0f);
}

void MotorController::ArmMotorsCallback()
{
	_motorsArmed = true;
}
