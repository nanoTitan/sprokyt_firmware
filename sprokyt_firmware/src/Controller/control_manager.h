#pragma once

#include <stdint.h>

enum CONTROL_STATE
{
	CONTROL_STATE_IDLE			= 0,
	CONTROL_STATE_CONNECTED		= 1,
	CONTROL_STATE_DISCONNECTED	= 2,
};

enum CONTROLLER_TYPE
{
	CONTROLLER_NONE				= 0,
	CONTROLLER_FLIGHT			= 1,
	CONTROLLER_ROVER			= 2,
	CONTROLLER_ROVER_BALANCE	= 3,
	CONTROLLER_ESC_PROGRAMMER	= 4,	
	
	CONTROLLER_COUNT			= 5
};

void ControlMgr_init();
void ControlMgr_setState(CONTROL_STATE state);
CONTROL_STATE ControlMgr_getState();
void ControlMgr_setType(CONTROLLER_TYPE ctrlType);
void ControlMgr_update();
void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
void ControlMgr_parseInstruction(uint8_t data_length, uint8_t *att_data);
