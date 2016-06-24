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
	PRINTF("%f, %f, %f\n", RadiansToDeg(yaw), RadiansToDeg(pitch), RadiansToDeg(roll));
	
	m_sfYaw = yaw;
	m_sfPitch = pitch;
	m_sfRoll = roll;
}

void FlightControl::UpdateBalance()
{
	
}

void FlightControl::UpdateMotor(uint8_t motorIndex, uint8_t value, uint8_t direction)
{
	MotorController::SetMotor(motorIndex, value, direction);
}