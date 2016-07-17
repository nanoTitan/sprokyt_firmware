#ifndef _ESC_PROGRAMMER_H_
#define _ESC_PROGRAMMER_H_

#include <stdint.h>

void EscProgrammer_init();
void EscProgrammer_update();
void EscProgrammer_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction);
void EscProgrammer_setInstruction(uint8_t instruction, uint8_t value);

#endif // _ESC_PROGRAMMER_H_