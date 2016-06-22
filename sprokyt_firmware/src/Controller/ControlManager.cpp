#include "ControlManager.h"


ControlManager::ControlManager()
:m_pController(NULL),
 m_currControllerType(CONTROLLER_NONE)
{
}

ControlManager::~ControlManager()
{
	DestroyController();
}

const ControlManager& ControlManager::Instance()
{
	static ControlManager* pInstance = NULL;
	if (!pInstance)
	{
		pInstance = new ControlManager();
	}
	
	return *pInstance;
}

void ControlManager::CreateController(CONTROLLER_TYPE cType)
{
	if (cType == m_currControllerType)
		return;
	
	DestroyController();
	
	switch (cType)
	{
		case CONTROLLER_FLIGHT:
			m_pController = new FlightControl();
			break;
		
		case CONTROLLER_ROVER:
			break;
		
		case CONTROLLER_ROVER_BALANCE:
			break;
			
		case CONTROLLER_NONE:
			break;
	}
}

const Controller* ControlManager::GetController()
{
	return m_pController;
}

void ControlManager::DestroyController()
{
	if (m_pController)
	{
		delete m_pController;
		m_pController = NULL;
	}
}