
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define MOTORS_ENABLED

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 1900
#define MIN_FLIGHT_THROTTLE 1100
#define MAX_FLIGHT_THROTTLE 1950

#define INSTRUCTION_THROTTLE 0
#define INSTRUCTION_YAW 1
#define INSTRUCTION_PITCH 2
#define INSTRUCTION_ROLL 3
#define INSTRUCTION_TRIM_THROTTLE 4
#define INSTRUCTION_TRIM_YAW 5
#define INSTRUCTION_TRIM_PITCH 6
#define INSTRUCTION_TRIM_ROLL 7

#define MC_NUM_MOTORS 4

// 250 Quad H Setup
#define MOTOR_A 0x01
#define MOTOR_B 0x02
#define MOTOR_C 0x04
#define MOTOR_D 0x08
#define MOTOR_ALL 0xFF

#define CTRL_UPDATE_TIME 1000

// 150 Quad X Setup
//#define MOTOR_D 0x04
//#define MOTOR_A 0x08
//#define MOTOR_B 0x01
//#define MOTOR_C 0x02
//#define MOTOR_ALL 0xFF

#endif /* _CONSTANTS_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
