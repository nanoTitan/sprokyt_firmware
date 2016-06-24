#pragma once

#include "FlightControl.h"

enum CONTROLLER_TYPE
{
	CONTROLLER_FLIGHT,
	CONTROLLER_ROVER,
	CONTROLLER_ROVER_BALANCE,
	CONTROLLER_NONE
};

class ControlManager
{
public:
	static ControlManager* Instance();
	~ControlManager();
	
	void CreateController(CONTROLLER_TYPE cType);
	Controller* GetController();
	void UpdateMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
	
private:
	ControlManager();
	void DestroyController();
	
	Controller* m_pController;
	CONTROLLER_TYPE m_currControllerType;
};