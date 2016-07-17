#pragma once

#include <stdint.h>

enum CONTROLLER_TYPE
{
	CONTROLLER_FLIGHT = 0,
	CONTROLLER_ROVER,
	CONTROLLER_ROVER_BALANCE,
	CONTROLLER_ESC_PROGRAMMER,
	CONTROLLER_NONE
};

void ControlMgr_init();
void ControlMgr_setType(CONTROLLER_TYPE ctrlType);
void ControlMgr_update();
void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
void ControlMgr_setInstruction(uint8_t instruction, uint8_t value);
void ControlMgr_connectionLost();