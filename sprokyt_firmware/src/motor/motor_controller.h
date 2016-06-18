// imu.h

//#include "mbed.h"

#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <stdint.h>

//namespace mbed {
//
//	class MotorController
//	{
//	public:
//		static void SetMotorA(uint8_t motorIndxMask, float power)
//		{
//		}
//	
//	private:
//	
//	};
//	
//}

void Init_MotorController(void);
void SetMotor(uint8_t motorIndxMask, float power);


#endif // _MOTOR_CONTROLLER_H_
