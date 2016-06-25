#include "FlightControl.h"
#include "MotorController.h"
#include "math_ext.h"
#include "debug.h"

FlightControl::FlightControl()
:m_sfYaw(0),
 m_sfPitch(0),
 m_sfRoll(0)
{
	m_pidArray[PID_PITCH_RATE].kP(0.7);
//  m_pidArray[PID_PITCH_RATE].kI(1);
	m_pidArray[PID_PITCH_RATE].imax(50);

	m_pidArray[PID_ROLL_RATE].kP(0.7);
	//  m_pidArray[PID_ROLL_RATE].kI(1);
	m_pidArray[PID_ROLL_RATE].imax(50);

	m_pidArray[PID_YAW_RATE].kP(2.5);
	//  m_pidArray[PID_YAW_RATE].kI(1);
	m_pidArray[PID_YAW_RATE].imax(50);

	m_pidArray[PID_PITCH_STAB].kP(4.5);
	m_pidArray[PID_ROLL_STAB].kP(4.5);
	m_pidArray[PID_YAW_STAB].kP(10);
}

FlightControl::~FlightControl()
{
}

void FlightControl::Update()
{
	UpdateBalance();
}

void FlightControl::UpdateIMU(float yaw, float pitch, float roll)
{
	PRINTF("%f, %f, %f\n", yaw, pitch, roll);
	
	// TODO: Gaurd this with a mutex for multithreading
	m_sfYaw = yaw;
	m_sfPitch = pitch;
	m_sfRoll = roll;
}

void FlightControl::UpdateBalance()
{
	uint8_t rcthr = 50;
	if (rcthr > 10)
	{   // *** MINIMUM THROTTLE TO DO CORRECTIONS MAKE THIS 20pts ABOVE YOUR MIN THR STICK ***/
		
		// TODO: Gaurd this with a mutex for multithreading
		long yaw_output   = m_pidArray[PID_YAW_RATE].get_pid(m_sfYaw - 0, 1);
		long pitch_output = m_pidArray[PID_PITCH_RATE].get_pid(m_sfPitch - 0, 1);  
		long roll_output  = m_pidArray[PID_ROLL_RATE].get_pid(m_sfRoll - 0, 1);  
		
		/*
		(A)--(B)
		   \/
		   /\
		(D)--(C)
		*/
		
		MotorController::SetMotor(MOTOR_A, rcthr - roll_output - pitch_output, 0);
		MotorController::SetMotor(MOTOR_D, rcthr - roll_output + pitch_output, 0);
		MotorController::SetMotor(MOTOR_B, rcthr + roll_output - pitch_output, 0);
		MotorController::SetMotor(MOTOR_C, rcthr + roll_output + pitch_output, 0);
	}
	else 
	{  
		// Turn motors off
		MotorController::SetMotor(MOTOR_ALL, 0, 0);
	
		for (int i = 0; i < 6; i++) // reset PID integrals whilst on the ground
			m_pidArray[i].reset_I();
	}
}

void FlightControl::UpdateMotor(uint8_t motorIndex, uint8_t value, uint8_t direction)
{
	MotorController::SetMotor(motorIndex, value, direction);
}