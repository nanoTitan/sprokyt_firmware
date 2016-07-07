#include "motor_controller.h"
#include "mbed.h"
#include "math_ext.h"

/* Private Variables ------------------------------------------------------------------*/
PwmOut _bldcArray[4] = { D1, D3, D9, D10 };
Timeout _motorArmTimeout;
bool _motorsArmed = false;

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();

void MotorController_init()
{	
	for (int i = 0; i < MC_NUM_MOTORS; ++i)
	{
		_bldcArray[i].period_us(20000);				// 50 hertz
		_bldcArray[i].pulsewidth_us(1000);			// 1000us - 2000us equals 0% - 100% power respectively
	}
}

void MotorController_setMotor(uint8_t motorIndxMask, float power, uint8_t direction)
{
	// 1000us - 2000us is 0% - 100% power respectively
	float x = clampf(power, 0, 1.0f);
	
	if (motorIndxMask & 0x01)
		_bldcArray[0].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x02)
		_bldcArray[1].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x04)
		_bldcArray[2].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x08)
		_bldcArray[3].pulsewidth_us(1000 + (1000 * x));	
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