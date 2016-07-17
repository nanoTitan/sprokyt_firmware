#include "control_manager.h"
#include "flight_control.h"
#include "esc_programmer.h"

enum CONTROLLER_TYPE m_currControllerType = CONTROLLER_FLIGHT;

void ControlMgr_setType(CONTROLLER_TYPE ctrlType)
{
	m_currControllerType = ctrlType;
}

void ControlMgr_init()
{
	FlightControl_init();
}

void ControlMgr_update()
{		
	if(m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_update();
	}
	else if(m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if(m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{		
	}
	else if(m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{		
	}	
}

void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction)
{
	if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_setMotor(motorIndex, value, direction);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if(m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{
		EscProgrammer_setMotor(motorIndex, value, direction);
	}
}

void ControlMgr_setInstruction(uint8_t instruction, uint8_t value)
{
	if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_setInstruction(instruction, value);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if(m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{		
		EscProgrammer_setInstruction(instruction, value);
	}
}

void ControlMgr_connectionLost()
{
	if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_connectionLost();
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if(m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{		
	}
}
