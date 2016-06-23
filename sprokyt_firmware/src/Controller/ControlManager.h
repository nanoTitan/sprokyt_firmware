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
	
private:
	ControlManager();
	void DestroyController();
	
	Controller* m_pController;
	CONTROLLER_TYPE m_currControllerType;
};