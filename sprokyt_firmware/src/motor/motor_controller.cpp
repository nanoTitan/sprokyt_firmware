#include "motor_controller.h"
#include "mbed.h"
#include "math_ext.h"

#if defined(MOTOR_STSPIN)
#include "STSpin.h"
#elif defined(MOTOR_TOSHIBA)
#include "TB6612FNG.h"
#endif


/* Private Variables ------------------------------------------------------------------*/
Timeout _motorArmTimeout;
bool _motorsArmed = false;

#if defined(MOTOR_ESC)
//PwmOut _bldcArray[4] = { D9, D10, PC_9, PC_8  };
#elif defined(MOTOR_TOSHIBA)
TB6612FNG motorDriver1(PB_1 /*PB_3*/, PA_1, PA_0, PB_0/*PA_7*/, PA_5, PA_6, PA_4);
TB6612FNG motorDriver2(PB_6, PA_15, PA_8, PC_7/*PB_3*/, PC_3, PC_4, PC_2);
#elif defined(MOTOR_STSPIN)
#endif

/* Private Functions ------------------------------------------------------------------*/
static void ArmMotorsCallback();
static void MotorController_armESCs();

#ifdef MOTOR_STSPIN
void STSpinInit();
void MotorController_setMotors_STSPIN(uint8_t motorIndxMask, float power, direction_t dir);
void STSPIN_setMotor(uint8_t indx, float pwm, uint8_t direction);
#elif defined(MOTOR_ESC)
void MotorController_setMotors_ESC(uint8_t motorIndxMask, float power, uint8_t direction);
#elif defined(MOTOR_TOSHIBA)
void MotorController_setMotors_TB6612(uint8_t motorIndxMask, float power, uint8_t direction);
#endif

void MotorController_init()
{	
#ifdef MOTOR_STSPIN	
	STSpinInit();
#elif defined(MOTOR_ESC)
//	_bldcArray[0].period_us(20000);				// 20000 us = 50 Hz, 2040.8us = 490 Hz, 83.3 us = 12 Khz	
//	_bldcArray[1].period_us(20000);
//	_bldcArray[2].period_us(20000);
//	_bldcArray[3].period_us(20000);
#elif defined(MOTOR_TOSHIBA)
	float fPwmPeriod = 0.00002f;      // 50KHz
	motorDriver1.setPwmAperiod(fPwmPeriod);
	motorDriver1.setPwmBperiod(fPwmPeriod);
	motorDriver2.setPwmAperiod(fPwmPeriod);
	motorDriver2.setPwmBperiod(fPwmPeriod);
#endif
	
	MotorController_setMotor(MOTOR_ALL, 0, BWD);
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

void MotorController_armESCs()
{
	for (int i = 1050; i < 2000; i += 50)
	{		
		MotorController_setMotor(MOTOR_ALL, i, FWD);
		wait_ms(10);
	}
	
	for (int i = 1950; i >= 1000; i -= 50)
	{
		MotorController_setMotor(MOTOR_ALL, i, FWD);
		wait_ms(10);
	}
	
	wait_ms(4000);
}

void MotorController_callibrateESCs()
{
	for (int i = 1050; i <= 2000; i+=50)
	{		
		MotorController_setMotor(MOTOR_D, i, FWD);
		wait_ms(10);
	}
	
	MotorController_setMotor(MOTOR_D, 2000, FWD);
	wait_ms(4000);
	
	for (int i = 1950; i >= 1000; i-=50)
	{
		MotorController_setMotor(MOTOR_D, i, FWD);
		wait_ms(10);
	}
	
	wait_ms(4000);
}

void MotorController_setMotor(uint8_t motorIndxMask, float power, direction_t dir)
{		
#ifdef MOTOR_STSPIN	
	MotorController_setMotors_STSPIN(motorIndxMask, power, dir);
#elif defined(MOTOR_ESC)
	MotorController_setMotors_ESC(motorIndxMask, power, dir);
#elif defined(MOTOR_TOSHIBA)
	MotorController_setMotors_TB6612(motorIndxMask, power, dir);
#endif
}

#ifdef MOTOR_STSPIN
void MotorController_setMotors_STSPIN(uint8_t motorIndxMask, float power, direction_t dir)
{
	float dutyCycle = map(power, 1000, 2000, 0, 100);		// Map between 0-100 where 50 is a 50% duty cycle	
	if (motorIndxMask & 0x01)
		STSpinSetMotor(0, 0, dutyCycle, dir);
	
	if (motorIndxMask & 0x02)
		STSpinSetMotor(0, 1, dutyCycle, dir);
	
	if (motorIndxMask & 0x04)
		STSpinSetMotor(1, 0, dutyCycle, dir);
	
	if (motorIndxMask & 0x08)
		STSpinSetMotor(1, 1, dutyCycle, dir);
}

#elif defined(MOTOR_ESC)
void MotorController_setMotor_ESC(uint8_t motorIndxMask, float power, uint8_t direction)
{
	// 1000us - 2000us is 0% - 100% power respectively
	float pwm = clampf(power, 0, 1.0f);	
	
	if (motorIndxMask & 0x01)
		_bldcArray[0].pulsewidth_us(power);	
	if (motorIndxMask & 0x02)
		_bldcArray[1].pulsewidth_us(power);	
	if (motorIndxMask & 0x04)
		_bldcArray[2].pulsewidth_us(power);	
	if (motorIndxMask & 0x08)
		_bldcArray[3].pulsewidth_us(power);	
}

#elif defined(MOTOR_TOSHIBA)
void MotorController_setMotors_TB6612(uint8_t motorIndxMask, float power, uint8_t direction)
{
	float pwm = map(power, 1000, 2000, 0, 1);		// Map between 0-1.0 where 0.5 is a 50% duty cycle	
	if (motorIndxMask & 0x01)
	{
		if (pwm == 0)
			motorDriver1.motorA_stop();
		else
		{
			motorDriver1.setPwmApulsewidth(pwm);
			if (direction == FWD)
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
			if (direction == FWD)
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
			if (direction == FWD)
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
			if (direction == FWD)
				motorDriver2.motorB_cw();
			else
				motorDriver2.motorB_ccw();
		}
	}
}
#endif