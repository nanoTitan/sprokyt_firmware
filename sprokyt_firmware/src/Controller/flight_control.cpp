#include "imu.h"
#include "flight_control.h"
#include "motor_controller.h"
#include "PID.h"
#include "math_ext.h"
#include "debug.h"

extern "C" {
#include "MotionFX_Manager.h"
}

#define YAW_RATE 0.5f
#define ROLL_RATE 0.5f
#define PITCH_RATE 0.5f
#define MIN_THROTTLE 1000
#define MAX_THROTTLE 1950

/* Private variables ---------------------------------------------------------*/
PID m_pidArray[PID_COUNT];
float m_rcThrottle = 0;
float m_rcYaw = 0;
float m_rcPitch = 0;
float m_rcRoll = 0;
float m_targetYaw = 0;
bool m_connectionLost = 0;

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnectionLost();

/* Private functions ---------------------------------------------------------*/
void FlightControl_init()
{	
	m_pidArray[PID_PITCH_RATE].kP(0.7f);
    //m_pidArray[PID_PITCH_RATE].kI(0.015);
	//m_pidArray[PID_PITCH_RATE].kD(0.01);
	m_pidArray[PID_PITCH_RATE].imax(50);

	m_pidArray[PID_ROLL_RATE].kP(0.7f);
	//m_pidArray[PID_ROLL_RATE].kI(0.04);	
	//m_pidArray[PID_ROLL_RATE].kD(0.01);
	m_pidArray[PID_ROLL_RATE].imax(50);

//	m_pidArray[PID_YAW_RATE].kP(0.02f);
//	m_pidArray[PID_YAW_RATE].kI(0.01f);
//	m_pidArray[PID_YAW_RATE].kD(0.01);
//	m_pidArray[PID_YAW_RATE].imax(50);	
	m_pidArray[PID_YAW_RATE].kP(0);
	m_pidArray[PID_YAW_RATE].kI(0);
	m_pidArray[PID_YAW_RATE].kD(0);
	m_pidArray[PID_YAW_RATE].imax(50);

	m_pidArray[PID_PITCH_STAB].kP(4.5f);
	m_pidArray[PID_ROLL_STAB].kP(4.5f);
	m_pidArray[PID_YAW_STAB].kP(10);
}


void FlightControl_update()
{	
	/*
		(A)--(B)
		   \/
		   /\
		(D)--(C)
	*/
	
	if (m_connectionLost)
	{
		UpdateConnectionLost();
		return;
	}
	
	// ACRO stabilization
	if (m_rcThrottle > 1200)
	{
		// *** MINIMUM THROTTLE TO DO CORRECTIONS MAKE THIS 20pts ABOVE YOUR MIN THR STICK ***/
		
		// RC Stability 
		float sfYaw = IMU_get_sf_yaw();
		float sfPitch = IMU_get_sf_pitch();
		float sfRoll = IMU_get_sf_roll();
		float yaw_stab_output   = clampf(m_pidArray[PID_YAW_STAB].get_pid(m_targetYaw - sfYaw, 1), -360, 360);
		float pitch_stab_output = clampf(m_pidArray[PID_PITCH_STAB].get_pid(m_rcPitch - sfPitch, 1), -250, 250);
		float roll_stab_output  = clampf(m_pidArray[PID_ROLL_STAB].get_pid(m_rcRoll - sfRoll, 1), -250, 250);
		
		// is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
		if (fabs(m_rcYaw) > 5) 
		{
			yaw_stab_output = m_rcYaw;
			m_targetYaw = sfYaw;			// remember this yaw for when pilot stops
		}
		
		// Rate PIDs
		osxMFX_input* pInput = MotionFX_manager_getDataIN();
		float bias[3] = {0, 0, 0};
		osx_MotionFX_getGbias(bias);
		float gx = pInput->gyro[0] - bias[0];
		float gy = pInput->gyro[1] - bias[1];
		float gz = pInput->gyro[2] - bias[2];
		
		float yaw_output   = clampf(m_pidArray[PID_YAW_RATE].get_pid(yaw_stab_output - gz, 1), -500, 500);
		float pitch_output = clampf(m_pidArray[PID_PITCH_RATE].get_pid(pitch_stab_output - gx, 1), -500, 500);
		float roll_output  = clampf(m_pidArray[PID_ROLL_RATE].get_pid(roll_stab_output - gy, 1), -500, 500);
		
		float powerA =  m_rcThrottle + roll_output + pitch_output - yaw_output;
		float powerB =  m_rcThrottle - roll_output + pitch_output + yaw_output;
		float powerC =  m_rcThrottle - roll_output - pitch_output - yaw_output;
		float powerD =  m_rcThrottle + roll_output - pitch_output + yaw_output;
		
		// Prevent throttle from reaching 2000us which causes ESC to go into programming mode
		powerA = clampf(powerA, powerA, MAX_THROTTLE);
		powerB = clampf(powerB, powerB, MAX_THROTTLE);
		powerC = clampf(powerC, powerC, MAX_THROTTLE);
		powerD = clampf(powerD, powerD, MAX_THROTTLE);
		
		static int cnt = 0;
		++cnt;
		if (cnt == 50)
		{
			//PRINTF("%d, %d, %d, %d\n", (int)roll_stab_output, (int)roll_output, (int)(m_rcThrottle + roll_output), (int)(m_rcThrottle - roll_output));
			//PRINTF("%d, %d, %d, %d\n", (int)yaw_stab_output, (int)yaw_output, (int)(m_rcThrottle + yaw_output), (int)(m_rcThrottle - yaw_output));
			//PRINTF("%d, %d, %d\n", (int)sfRoll, (int)roll_stab_output, (int)roll_output);
			//PRINTF("%d, %d, %d\n", (int)sfYaw, (int)yaw_stab_output, (int)yaw_output);			
			PRINTF("%d, %d, %d\n", (int)sfYaw, (int)sfPitch, (int)sfRoll);					// yaw, pitch, roll
			//PRINTF("%d, %d, %d, %d\n", (int)powerA, (int)powerB, (int)powerC, (int)powerD);		// A, B, C, D
			cnt = 0;
		}
		
		// MEMS facing Forwards
		MotorController_setMotor(MOTOR_A, powerA, 0);
		MotorController_setMotor(MOTOR_B, powerB, 0);
		MotorController_setMotor(MOTOR_C, powerC, 0);		
		MotorController_setMotor(MOTOR_D, powerD, 0);
	}
	else
	{
		// Turn motors off
		MotorController_setMotor(MOTOR_ALL, 1000, 0);
		
		// Reset target yaw for next takeoff
		m_targetYaw = IMU_get_sf_yaw();			
	
		for (int i = 0; i < PID_COUNT; ++i) // reset PID integrals whilst on the ground
			m_pidArray[i].reset_I();
	}
}

void UpdateConnectionLost()
{
	// TODO: Show flashing LEDs if connection is lost
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
		//m_rcRoll *= -1;
	}
}

void FlightControl_connectionLost()
{
	m_rcThrottle = 0.0f;	
	MotorController_setMotor(MOTOR_ALL, m_rcThrottle, 0);
	m_connectionLost = true;
}