#include "motor_controller.h"
#include "mbed.h"
#include "math.h"

/*
Arduino Name	-	Motor Index
D3				-	0
D5				-	1
D6				-	2
D9				-	3
*/

#define MC_NUM_MOTORS 4

/* Private Variables ------------------------------------------------------------------*/
PwmOut _bldcArray[4] = { D3, D5, D6, D9 };
PwmOut _bldc0(D9);
Timeout _motorArmTimeout;
bool _motorsArmed = false;

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotors();
static void MotorTimeout();
static void ArmMotorsCallback();

void Init_MotorController(void)
{
	_bldc0.period_us(20000);						// 50 hertz
	_bldc0.pulsewidth_us(1000);						// 1000us - 2000us is 0% - 100% power respectively
	
	for (int i = 0; i < MC_NUM_MOTORS; ++i)
	{
		_bldcArray[i].period_us(20000);						// 50 hertz
	}
	
	ArmMotors();
}

void SetMotor(uint8_t motorIndxMask, float power)
{
	// 1000us - 2000us is 0% - 100% power respectively
	float x = clamp(power, 0.0f, 1.0f);
	
	if (motorIndxMask & 0x01)
		_bldcArray[0].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x02)
		_bldcArray[1].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x04)
		_bldcArray[2].pulsewidth_us(1000 + (1000 * x));	
	if (motorIndxMask & 0x08)
		_bldcArray[3].pulsewidth_us(1000 + (1000 * x));	
}

void ArmMotors()
{
	for (int i = 0; i < MC_NUM_MOTORS; ++i)
	{
		_bldcArray[i].pulsewidth_us(1000);		// 1000us arms motors
	}
	
	_motorArmTimeout.attach(ArmMotorsCallback, 3.0f);
}

void ArmMotorsCallback()
{
	_motorsArmed = true;
}