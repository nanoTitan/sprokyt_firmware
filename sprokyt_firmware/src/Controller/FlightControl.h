#pragma once

#include "Controller.h"
#include "PID.h"

class FlightControl: public Controller
{
public:
	FlightControl();
	~FlightControl();
	
	void Update();
	void UpdateIMU(float yaw, float pitch, float roll);
	void UpdateMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
	
private:
	void UpdateBalance();	
	
	PID m_pidArray[PID_COUNT];
	float m_sfYaw;
	float m_sfPitch;
	float m_sfRoll;
	bool m_doUpdate;
};
