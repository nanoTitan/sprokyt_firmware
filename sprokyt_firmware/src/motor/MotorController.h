#pragma once

#include <stdint.h>

class MotorController
{
public:
	static void InitMotors();
	static void SetMotor(unsigned char motorIndxMask, float power);
	
private:
	static void ArmMotors();
	static void MotorTimeout();
	static void ArmMotorsCallback();
	
};

//void Init_MotorController(void);
//void SetMotor(uint8_t motorIndxMask, float power);
