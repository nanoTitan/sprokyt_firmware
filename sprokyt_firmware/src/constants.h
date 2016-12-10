
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define FIRMWARE_VERSION "0.2.0"

#define WIFI_PING_TIMEOUT 3600

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define MIN_FLIGHT_THROTTLE 1100
#define MAX_FLIGHT_THROTTLE 1950
#define HOVER_FLIGHT_THROTTLE 1500
#define DESCENT_FLIGHT_THROTTLE 1400

#define INSTRUCTION_PING			0
#define INSTRUCTION_CONTROL_TYPE	1
#define INSTRUCTION_THROTTLE		10
#define INSTRUCTION_YAW				11
#define INSTRUCTION_PITCH			12
#define INSTRUCTION_ROLL			13
#define INSTRUCTION_TRIM_THROTTLE	14
#define INSTRUCTION_TRIM_YAW		15
#define INSTRUCTION_TRIM_PITCH		16
#define INSTRUCTION_TRIM_ROLL		17
#define INSTRUCTION_YAW_PID			18
#define INSTRUCTION_PITCH_PID		19
#define INSTRUCTION_ROLL_PID		20

#define MOTORS_ENABLED
#define MC_NUM_MOTORS 4

//#define MOTOR_ESC
//#define MOTOR_TOSHIBA
#define MOTOR_STSPIN

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

// Debugging
#define DEBUG_FLIGHT_CONTROL_NO_CONNECT

/**
     * @brief Rotation modes.
     */
typedef enum
{
	BWD = 0, /* Backward. */
	FWD = 1  /* Forward. */
} direction_t;

#endif /* _CONSTANTS_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
