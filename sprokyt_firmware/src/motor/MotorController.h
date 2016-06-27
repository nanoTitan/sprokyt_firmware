#pragma once

#include <stdint.h>

/*
Arduino Name	-	Motor Index
D3				-	0
D5				-	1
D6				-	2
D9				-	3

Motor Name to Mask
A	0x1
B	0x2
C	0x4
D	0x8
*/

#define MC_NUM_MOTORS 4
#define MOTOR_A 0x01
#define MOTOR_B 0x02
#define MOTOR_C 0x04
#define MOTOR_D 0x08
#define MOTOR_ALL 0xFF

class MotorController
{
public:
	static void InitMotors();
	static void SetMotor(uint8_t motorIndxMask, uint8_t power, uint8_t direction);
	
private:
	static void MotorTimeout();
	static void ArmMotorsCallback();
	
};

//void Init_MotorController(void);
//void SetMotor(uint8_t motorIndxMask, float power);
