#include "FlightControl.h"
#include "math_ext.h"
#include "debug.h"

FlightControl::FlightControl()
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

void FlightControl::Update(float yaw, float pitch, float roll)
{
	PRINTF("pitch: %f, roll: %f, yaw: %f\n", RadiansToDeg(yaw), RadiansToDeg(pitch), RadiansToDeg(roll));
}
