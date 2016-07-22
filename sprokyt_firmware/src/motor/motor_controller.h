#pragma once

#include <stdint.h>

void MotorController_init();
void MotorController_setMotor(uint8_t motorIndxMask, float power, uint8_t direction);
void MotorController_armMotors();
bool MotorController_isArmed();
void MotorController_callibrateESCs();