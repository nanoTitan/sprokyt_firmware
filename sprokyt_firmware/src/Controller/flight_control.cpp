#include "imu.h"
#include "flight_control.h"
#include "motor_controller.h"
#include "constants.h"
#include "PID.h"
//#include "BLE.h"
#include "Wifi.h"
#include "math_ext.h"
#include "debug.h"
#include <mbed.h>

extern "C" {
#include "MotionFX_Manager.h"
}

enum FLIGHT_MODE
{
	RATE_MODE = 1,
	STABILITY_MODE = 0
};

/* Private variables ---------------------------------------------------------*/
PID m_pidArray[PID_COUNT];
float m_rcThrottle = 0;
float m_rcYaw = 0;
float m_rcPitch = 0;
float m_rcRoll = 0;
float m_trimThrottle = 0;
float m_trimYaw = 0;
float m_trimPitch = 0;
float m_trimRoll = 0;
float m_targetYaw = 0;
float m_trimScale = 10.0f;
FLIGHT_MODE m_flightMode = STABILITY_MODE;

Timer fcTimer;

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnectionLost();
static void Disarm();
void FlightControl_setPIDValue(int index, const PIDInfo& info);

/* Private functions ---------------------------------------------------------*/
void FlightControl_init()
{	
	m_pidArray[PID_YAW_RATE].kP(0.0);
	m_pidArray[PID_YAW_RATE].kI(0);
	m_pidArray[PID_YAW_RATE].kD(0);
	m_pidArray[PID_YAW_RATE].imax(50);
	
	m_pidArray[PID_PITCH_RATE].kP(1.4f);
	m_pidArray[PID_PITCH_RATE].kI(0.0f);
	m_pidArray[PID_PITCH_RATE].kD(0);
	m_pidArray[PID_PITCH_RATE].imax(50);

	m_pidArray[PID_ROLL_RATE].kP(1.4f);		//	Rate only - 1.7f
	m_pidArray[PID_ROLL_RATE].kI(0.0f);	
	m_pidArray[PID_ROLL_RATE].kD(0);
	m_pidArray[PID_ROLL_RATE].imax(50);

	m_pidArray[PID_YAW_STAB].kP(5.0f);		// 10
	m_pidArray[PID_PITCH_STAB].kP(3.0f);	// 4.5f
	m_pidArray[PID_ROLL_STAB].kP(3.5f);		// 4.5f
	
	fcTimer.start();
}


void FlightControl_update()
{		
	// Don't update motors if we aren't in a connection lost state and without valid throttle values
	if (m_rcThrottle == 0)
	{
		// Reset target yaw for next takeoff
		m_targetYaw = IMU_get_sf_yaw();	
		return;
	}
	
	//if (!BLE::IsConnected())
	if(!Wifi::Instance()->IsConnected())
	{
		if (m_rcThrottle > MIN_FLIGHT_THROTTLE)
			UpdateConnectionLost();
		
		if (m_rcThrottle <= MIN_FLIGHT_THROTTLE)
			Disarm();
		
		return;
	}
	
	if (m_rcThrottle > MIN_FLIGHT_THROTTLE)
	{
		// Prevent the controller from updating the ESCs too often
		if (fcTimer.read_us() < CTRL_UPDATE_TIME)
			return;
		
		fcTimer.reset();
		
		float yaw_stab_output   = 0;
		float pitch_stab_output = 0;
		float roll_stab_output  = 0;
		float sfYaw = IMU_get_sf_yaw();
		float sfPitch = IMU_get_sf_pitch();
		float sfRoll = IMU_get_sf_roll();
		
		if (m_flightMode == STABILITY_MODE)
		{			
			// RC Stability 
			yaw_stab_output   = clampf(m_pidArray[PID_YAW_STAB].get_pid(wrap_180(m_targetYaw - sfYaw), 1), -360, 360);
			pitch_stab_output = clampf(m_pidArray[PID_PITCH_STAB].get_pid(m_rcPitch - sfPitch, 1), -250, 250);
			roll_stab_output  = clampf(m_pidArray[PID_ROLL_STAB].get_pid(m_rcRoll - sfRoll, 1), -250, 250);
		
			// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
			if (fabs(m_rcYaw) > 5) 
			{
				yaw_stab_output = m_rcYaw;
				m_targetYaw = sfYaw;			// remember this yaw for when pilot stops
			}
		}
		else
		{
			yaw_stab_output = m_rcYaw;
			m_targetYaw = sfYaw;			// remember this yaw for when pilot stops
			
			pitch_stab_output = m_rcPitch;
			roll_stab_output = m_rcRoll;
		}
		
		// Rate PIDs
		osxMFX_input* pInput = MotionFX_manager_getDataIN();
		float bias[3] = {0, 0, 0};
		osx_MotionFX_getGbias(bias);
		float gx = pInput->gyro[0] - bias[0];
		float gy = pInput->gyro[1] - bias[1];
		float gz = pInput->gyro[2] - bias[2];
		
		osxMFX_output* pOutput = MotionFX_manager_getDataOUT();
		float heading = pOutput->heading_9X;
		
		float yaw_output   = clampf(m_pidArray[PID_YAW_RATE].get_pid(yaw_stab_output + gz, 1), -500, 500);
		float pitch_output = clampf(m_pidArray[PID_PITCH_RATE].get_pid(pitch_stab_output + gx, 1), -500, 500);
		float roll_output  = clampf(m_pidArray[PID_ROLL_RATE].get_pid(roll_stab_output + gy, 1), -500, 500);
		
		// Trim
		float trimA = m_trimThrottle + m_trimRoll - m_trimPitch + m_trimYaw;
		float trimB = m_trimThrottle - m_trimRoll - m_trimPitch - m_trimYaw;
		float trimC = m_trimThrottle - m_trimRoll + m_trimPitch + m_trimYaw;
		float trimD = m_trimThrottle + m_trimRoll + m_trimPitch - m_trimYaw;
		
		/*
		(A)--(B)
		   \/
		   /\
		(D)--(C)
		*/
		float powerA =  m_rcThrottle + roll_output + pitch_output + yaw_output + trimA;
		float powerB =  m_rcThrottle - roll_output + pitch_output - yaw_output + trimB;
		float powerC =  m_rcThrottle - roll_output - pitch_output + yaw_output + trimC;
		float powerD =  m_rcThrottle + roll_output - pitch_output - yaw_output + trimD;
		
		/*
		   (A)
		    |
		(C)---(D)
			|
		   (B)		
		*/
//		float powerA =  m_rcThrottle + pitch_output - yaw_output;
//		float powerB =  m_rcThrottle - pitch_output - yaw_output;
//		float powerC =  m_rcThrottle + roll_output + yaw_output;
//		float powerD =  m_rcThrottle - roll_output + yaw_output;
		
		// Prevent throttle from reaching 2000us which causes ESC to go into programming mode
		powerA = clampf(powerA, MIN_FLIGHT_THROTTLE, MAX_FLIGHT_THROTTLE);
		powerB = clampf(powerB, MIN_FLIGHT_THROTTLE, MAX_FLIGHT_THROTTLE);
		powerC = clampf(powerC, MIN_FLIGHT_THROTTLE, MAX_FLIGHT_THROTTLE);
		powerD = clampf(powerD, MIN_FLIGHT_THROTTLE, MAX_FLIGHT_THROTTLE);
		
		static int cnt = 0;
		++cnt;
		if (cnt == 50)
		{
			//PRINTF("%d, %d, %d, %d\r\n", (int)m_rcThrottle, (int)m_rcYaw, (int)m_rcPitch, (int)m_rcRoll);
			//PRINTF("%d, %d, %d, %d\r\n", (int)sfRoll, (int)roll_stab_output, (int)gy, (int)roll_output);
			//PRINTF("%d, %d, %d, %d\r\n", (int)sfPitch, (int)pitch_stab_output, (int)gx, (int)pitch_output);
			//PRINTF("%d, %d, %d, %d\r\n", (int)sfYaw, (int)yaw_stab_output, (int)gz, (int)yaw_output);
			//PRINTF("%.2f, %.2f\r\n", sfYaw, heading);					// yaw, pitch, roll
			//PRINTF("%.2f, %d, %d\r\n", heading, (int)sfPitch, (int)sfRoll);					// yaw, pitch, roll
			//PRINTF("%.2f, %.2f, %.2f\r\n", pInput->mag[0], pInput->mag[1], pInput->mag[2]);
			PRINTF("%d, %d, %d, %d\r\n", (int)powerA, (int)powerB, (int)powerC, (int)powerD);		// A, B, C, D
			cnt = 0;
		}
		
		// Set motor speed
#if defined(MOTORS_ENABLED)
		MotorController_setMotor(MOTOR_A, powerA, DIR_CW);
		MotorController_setMotor(MOTOR_B, powerB, DIR_CCW);
		MotorController_setMotor(MOTOR_C, powerC, DIR_CW);		
		MotorController_setMotor(MOTOR_D, powerD, DIR_CCW);
#endif // MOTORS_ENABLED
	}
	else
	{
#if defined(MOTORS_ENABLED)
		// Allow ESCs to be armed or turn motors on/off
		MotorController_setMotor(MOTOR_ALL, m_rcThrottle, 0);
#endif // MOTORS_ENABLED
		
		// Reset target yaw for next takeoff
		m_targetYaw = IMU_get_sf_yaw();			
		
		for (int i = 0; i < PID_COUNT; ++i) // reset PID integrals if on the ground
			m_pidArray[i].reset_I();
	}
}

void UpdateConnectionLost()
{
	// TODO: Show flashing LEDs if connection is lost
	
	// Power down motors slowly
	
	m_rcThrottle -= 0.5f;
	if (m_rcThrottle < MIN_FLIGHT_THROTTLE)
	{
		m_rcThrottle = MIN_FLIGHT_THROTTLE;
	}
	
	MotorController_setMotor(MOTOR_ALL, m_rcThrottle, 0);	
	//PRINTF("power down %d\n", (int)m_rcThrottle);	
}

void Disarm()
{	
	// Turn motors off
	MotorController_setMotor(MOTOR_ALL, MIN_THROTTLE, 0);
	m_rcThrottle = 0;
}

void FlightControl_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	if (data_length == 2)
	{
		uint8_t instruction = att_data[0];
		uint8_t value = att_data[1];
		
		FlightControl_setInstruction(instruction, value);
	}
	else if (data_length == 4)
	{
		uint8_t instruction = att_data[0];
		PIDInfo info;
		float pidScale = 0.1f;
		
		info.P	= (float)att_data[1] * pidScale;
		info.I	= (float)att_data[2] * pidScale;
		info.D	= (float)att_data[3] * pidScale;
		
		FlightControl_setPIDValue(instruction, info);
	}
}

void FlightControl_setPIDValue(int index, const PIDInfo& info)
{
	if (index == PID_YAW_RATE)
	{
		m_pidArray[PID_YAW_RATE].kP(info.P);
		m_pidArray[PID_YAW_RATE].kI(info.I);
		m_pidArray[PID_YAW_RATE].kD(info.D);
	}
	else if (index == PID_PITCH_RATE)
	{
		m_pidArray[PID_PITCH_RATE].kP(info.P);
		m_pidArray[PID_PITCH_RATE].kI(info.I);	
		m_pidArray[PID_PITCH_RATE].kD(info.D);
	}
	else if (index == PID_ROLL_RATE)
	{
		m_pidArray[PID_ROLL_RATE].kP(info.P);
		m_pidArray[PID_ROLL_RATE].kI(info.I);
		m_pidArray[PID_ROLL_RATE].kD(info.D);
	}
	
	// Reset target yaw for next takeoff
	m_targetYaw = IMU_get_sf_yaw();	
}

void FlightControl_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction)
{
	MotorController_setMotor(motorIndex, value, direction);
}

void FlightControl_setInstruction(uint8_t instruction, uint8_t value)
{
	if(instruction == INSTRUCTION_THROTTLE)
	{
		m_rcThrottle = map(value, 0, 255, MIN_THROTTLE, MAX_THROTTLE);
	}
	else if (instruction == INSTRUCTION_YAW)
	{
		int8_t adjValue = (int8_t)value;
		m_rcYaw = map(adjValue, -128, 127, -150, 150);
	}
	else if (instruction == INSTRUCTION_PITCH)
	{
		int8_t adjValue = (int8_t)value;
		m_rcPitch = map(adjValue, -128, 127, -45, 45);
		m_rcPitch *= -1;
	}
	else if (instruction == INSTRUCTION_ROLL)
	{
		int8_t adjValue = (int8_t)value;
		m_rcRoll = map(adjValue, -128, 127, -45, 45);
	}
	else if (instruction == INSTRUCTION_TRIM_THROTTLE)
	{
		int8_t adjValue = (int8_t)value;
		m_trimThrottle = (float)adjValue * m_trimScale;
	}
	else if (instruction == INSTRUCTION_TRIM_YAW)
	{
		int8_t adjValue = (int8_t)value;
		m_trimYaw = (float)adjValue * m_trimScale;
	}
	else if (instruction == INSTRUCTION_TRIM_PITCH)
	{
		int8_t adjValue = (int8_t)value;
		m_trimPitch = (float)adjValue * m_trimScale;
	}
	else if (instruction == INSTRUCTION_TRIM_ROLL)
	{
		int8_t adjValue = (int8_t)value;
		m_trimRoll = (float)adjValue * m_trimScale;
	}
}

void FlightControl_connectionLost()
{
}