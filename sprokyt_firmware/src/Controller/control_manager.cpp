#include "control_manager.h"
#include "PID.h"
#include "Wifi.h"
#include "user_control.h"
#include "flight_control.h"
#include "esc_programmer.h"
#include "debug.h"

/* Private variables ---------------------------------------------------------*/
enum CONTROL_STATE m_controlState = CONTROL_STATE_IDLE;
enum CONTROLLER_TYPE m_currControllerType = CONTROLLER_FLIGHT;
uint32_t m_lastPing = 0;

/* Private function prototypes -----------------------------------------------*/
static void ControlMgr_setInstruction(uint8_t instruction, uint8_t value);
static void ResetPing();

CONTROL_STATE ControlMgr_getState()
{
	return m_controlState;
}

void ControlMgr_setState(CONTROL_STATE state)
{
	m_controlState = state;
}

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
	if (ControlMgr_getState() == CONTROL_STATE_CONNECTED)
	{
		uint32_t delta = HAL_GetTick() - m_lastPing;
		if (delta > WIFI_PING_TIMEOUT)							// Ping check
		{
			ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
		}
		else if (!Wifi::Instance()->IsConnected())
			ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
	}
	
	switch(m_currControllerType)
	{
		case CONTROLLER_USER:
			break;
		
		case CONTROLLER_FLIGHT:
			FlightControl_update();
		
		case CONTROLLER_ROVER:
			break;
		
		case CONTROLLER_ROVER_BALANCE:
			break;
		
		case CONTROLLER_ESC_PROGRAMMER:
			break;
		
		default:
			break;
	}
}

void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, direction_t dir)
{
	if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_setMotor(motorIndex, value, dir);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if(m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{
		EscProgrammer_setMotor(motorIndex, value, dir);
	}
}

void ControlMgr_parseInstruction(uint8_t data_length, uint8_t *att_data)
{	
	if (data_length == 0)
		return;
	
	// Check for ping
	uint8_t instruction = att_data[0];
	if (instruction == INSTRUCTION_PING)
	{
		ResetPing();		
		return;
	}
	
	// Check for controller type change
	if (instruction == INSTRUCTION_CONTROL_TYPE)
	{
		if (data_length != 2)
		{
			PRINTF("Error: Instruction length is not 2\r\n");	
			return;
		}	
		
		uint8_t type = att_data[1];
		if (type < CONTROLLER_COUNT)
			ControlMgr_setType((CONTROLLER_TYPE)type);
		
		ResetPing();
		return;
	}
	
	// Update based on controller type
	if(m_currControllerType == CONTROLLER_USER)
	{
		UserControl_parseInstruction(data_length, att_data);
	}
	else if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_parseInstruction(data_length, att_data);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if (m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{
	}
	
	ResetPing();
}

void ResetPing()
{
	ControlMgr_setState(CONTROL_STATE_CONNECTED);
	PRINTF("dT: %lu\r\n", HAL_GetTick() - m_lastPing);	
	m_lastPing = HAL_GetTick();
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
