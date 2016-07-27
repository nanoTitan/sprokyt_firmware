#include "control_manager.h"
#include "PID.h"
#include "flight_control.h"
#include "esc_programmer.h"

// Private variables
enum CONTROLLER_TYPE m_currControllerType = CONTROLLER_FLIGHT;

// Private functions
static void ControlMgr_setInstruction(uint8_t instruction, uint8_t value);

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
		EscProgrammer_update();
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

void ControlMgr_parseInstruction(uint8_t data_length, uint8_t *att_data)
{
	if (data_length == 2)
	{
		uint8_t instruction = att_data[0];
		uint8_t value = att_data[1];
		ControlMgr_setInstruction(instruction, value);
	}
	else if (data_length == 10)
	{
		uint8_t instruction = att_data[0];
		PIDInfo yawInfo;
		PIDInfo pitchInfo;
		PIDInfo rollInfo;
		
		yawInfo.P = att_data[1];
		yawInfo.I = att_data[2];
		yawInfo.D = att_data[3];
		pitchInfo.P = att_data[4];
		pitchInfo.I = att_data[5];
		pitchInfo.D = att_data[6];
		rollInfo.P = att_data[7];
		rollInfo.I = att_data[8];
		rollInfo.D = att_data[9];
		
		FlightControl_setPIDValues(yawInfo, pitchInfo, rollInfo);
		ControlMgr_setType(CONTROLLER_FLIGHT);
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
