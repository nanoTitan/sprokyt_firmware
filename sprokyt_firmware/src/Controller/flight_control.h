#ifndef _FLIGHT_CONTROL_H_
#define _FLIGHT_CONTROL_H_

#include <stdint.h>

struct PIDInfo;

void FlightControl_init();
void FlightControl_update();
void FlightControl_parseInstruction(uint8_t data_length, uint8_t *att_data);
void FlightControl_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
void FlightControl_setInstruction(uint8_t instruction, uint8_t value);
void FlightControl_connectionLost();

#endif // _FLIGHT_CONTROL_H_