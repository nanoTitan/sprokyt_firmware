#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_

#include <stdint.h>

void FlightControl_init();
void FlightControl_update();
void FlightControl_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
void FlightControl_setInstruction(uint8_t instruction, uint8_t value);
void FlightControl_connectionLost();

#endif // _FLIGHT_CONTROL_H_