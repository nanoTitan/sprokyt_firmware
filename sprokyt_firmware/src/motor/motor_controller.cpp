#include "motor_controller.h"
#include "mbed.h"
#include "TB6612FNG.h"
#include "constants.h"
#include "math_ext.h"

/* Private Variables ------------------------------------------------------------------*/
//PwmOut _bldcArray[4] = { D9, D10, PC_9, PC_8  };
TB6612FNG motorDriver1(PB_10, PA_1, PA_0, PA_7, PA_5, PA_6, PA_4);
TB6612FNG motorDriver2(PB_6, PA_15, PA_8, PC_7/*PB_3*/, PC_3, PC_4, PC_2);
Timeout _motorArmTimeout;
bool _motorsArmed = false;

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();
static void MotorController_armESCs();

void MotorController_init()
{	
//	_bldcArray[0].period_us(20000);				// 20000 us = 50 Hz, 2040.8us = 490 Hz, 83.3 us = 12 Khz	
//	_bldcArray[1].period_us(20000);
//	_bldcArray[2].period_us(20000);
//	_bldcArray[3].period_us(20000);
	
	float fPwmPeriod = 0.00002f;      // 50KHz
	motorDriver1.setPwmAperiod(fPwmPeriod);
	motorDriver1.setPwmBperiod(fPwmPeriod);
	motorDriver2.setPwmAperiod(fPwmPeriod);
	motorDriver2.setPwmBperiod(fPwmPeriod);
	MotorController_setMotor(MOTOR_ALL, 0, DIR_CW);
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
	//float pwm = clampf(power, 0, 1.0f);		
	
	float pwm = map(power, 1000, 2000, 0, 1);		// Map between 0-1.0 where 0.5 is a 50% duty cycle
	
//	if (motorIndxMask & 0x01)
//		_bldcArray[0].pulsewidth_us(power);	
//	if (motorIndxMask & 0x02)
//		_bldcArray[1].pulsewidth_us(power);	
//	if (motorIndxMask & 0x04)
//		_bldcArray[2].pulsewidth_us(power);	
//	if (motorIndxMask & 0x08)
//		_bldcArray[3].pulsewidth_us(power);	
	
	
	if (motorIndxMask & 0x01)
	{
		if (pwm == 0)
			motorDriver1.motorA_stop();
		else
		{
			motorDriver1.setPwmApulsewidth(pwm);
			if (direction == DIR_CW)
				motorDriver1.motorA_cw();
			else
				motorDriver1.motorA_ccw();
		}		
	}
	
	if (motorIndxMask & 0x02)
	{
		if (pwm == 0)
			motorDriver1.motorB_stop();
		else
		{
			motorDriver1.setPwmBpulsewidth(pwm);	
			if (direction == DIR_CW)
				motorDriver1.motorB_cw();
			else
				motorDriver1.motorB_ccw();
		}
	}
	
	if (motorIndxMask & 0x04)
	{
		if (pwm == 0)
			motorDriver2.motorA_stop();
		else
		{
			motorDriver2.setPwmApulsewidth(pwm);	
			if (direction == DIR_CW)
				motorDriver2.motorA_cw();
			else
				motorDriver2.motorA_ccw();
		}
	}
	
	if (motorIndxMask & 0x08)
	{
		if (pwm == 0)
			motorDriver2.motorB_stop();
		else
		{
			motorDriver2.setPwmBpulsewidth(pwm);	
			if (direction == DIR_CW)
				motorDriver2.motorB_cw();
			else
				motorDriver2.motorB_ccw();
		}
	}
}

void MotorController_armMotors()
{
//	for (int i = 0; i < MC_NUM_MOTORS; ++i)
//	{
//		_bldcArray[i].pulsewidth_us(1000);			// 1000us - 2000us equals 0% - 100% power respectively
//	}
	
	_motorsArmed = true;
}

bool MotorController_isArmed()
{
	return _motorsArmed;
}