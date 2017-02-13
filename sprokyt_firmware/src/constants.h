
#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#define FIRMWARE_VERSION_0_2_2
//#define FIRMWARE_VERSION_0_2_3

#define FIRMWARE_VERSION_STR "0.2.2"

#define WIFI_PING_TIMEOUT 3600

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define MIN_FLIGHT_THROTTLE 1100
#define MAX_FLIGHT_THROTTLE 1950
#define HOVER_FLIGHT_THROTTLE 1500
#define DESCENT_FLIGHT_THROTTLE 1400

#define INSTRUCTION_PING			0
#define INSTRUCTION_CONTROL_TYPE	1
#define	INSTRUCTION_FORWARD			2
#define	INSTRUCTION_BACKWARD		3
#define INSTRUCTION_THROTTLE		100
#define INSTRUCTION_YAW				101
#define INSTRUCTION_PITCH			102
#define INSTRUCTION_ROLL			103
#define INSTRUCTION_TRIM_THROTTLE	104
#define INSTRUCTION_TRIM_YAW		105
#define INSTRUCTION_TRIM_PITCH		106
#define INSTRUCTION_TRIM_ROLL		107
#define INSTRUCTION_YAW_PID			108
#define INSTRUCTION_PITCH_PID		109
#define INSTRUCTION_ROLL_PID		110

#define MOTORS_ENABLED
#define MC_NUM_MOTORS 4

//#define MOTOR_ESC
#define MOTOR_TOSHIBA
//#define MOTOR_STSPIN

// 250 Quad H Setup
#define MOTOR_A 0x01
#define MOTOR_B 0x02
#define MOTOR_C 0x04
#define MOTOR_D 0x08
#define MOTOR_ALL 0xFF

#define CTRL_UPDATE_TIME 1000

//  Default supply voltage in mV
#ifndef ADC_SUPPLY_VOLTAGE
#define ADC_SUPPLY_VOLTAGE		3300
#endif

// Multipliers for VBAT measurement
#if defined (STM32F40_41xxx)
#define ADC_VBAT_MULTI			2
#endif
#if defined (STM32F427_437xx) || defined (STM32F429_439xx) || defined (STM32F401xx) || defined (STM32F411xE)
#define ADC_VBAT_MULTI			4
#endif


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
