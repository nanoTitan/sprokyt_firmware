#include "esc_programmer.h"
#include "motor_controller.h"
#include "math_ext.h"
#include "debug.h"

/* Private function prototypes -----------------------------------------------*/
static void UpdateConnectionLost();

/* Private functions ---------------------------------------------------------*/
void EscProgrammer_init()
{
	
}


void EscProgrammer_update()
{	
}

void EscProgrammer_setMotor(uint8_t motorIndex, uint8_t value, uint8_t direction)
{
	MotorController_setMotor(motorIndex, value, direction);
}

void EscProgrammer_setInstruction(uint8_t instruction, uint8_t value)
{
	static int motor = MOTOR_A;
	if(instruction == INSTRUCTION_THROTTLE)
	{
		float throttle = map(value, 0, 255, 1000, 2000);
		MotorController_setMotor(motor, throttle, 0);		
	}
}