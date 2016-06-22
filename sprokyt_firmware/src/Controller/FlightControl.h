#pragma once

#include "Controller.h"
#include "PID.h"

class FlightControl: public Controller
{
public:
	FlightControl();
	~FlightControl();
	
	void Update(float yaw, float pitch, float roll);
	
private:
	
	PID m_pidArray[PID_COUNT];
};