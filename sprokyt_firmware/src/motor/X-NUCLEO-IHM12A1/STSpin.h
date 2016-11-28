#pragma once

#include <stdint.h>
#include "constants.h"

void STSpinInit();
void STSpinDestroy();
void STSpinSetMotor(uint8_t deviceIndx, uint8_t motorIndx, float dutyCycle, direction_t dir);
