#include "STSpin.h"
#include "error.h"
#include "debug.h"
#include "stspin240_250_class.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Initialization parameters of the motor connected to the expansion board. */
Stspin240_250_Init_t initDeviceParameters =
{
	20000, /* Frequency of PWM of Input Bridge A in Hz up to 100000Hz             */
	20000, /* Frequency of PWM of Input Bridge B in Hz up to 100000Hz             */
	20000, /* Frequency of PWM used for Ref pin in Hz up to 100000Hz              */
	50,    /* Duty cycle of PWM used for Ref pin (from 0 to 100)                  */
	TRUE   /* Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc) */
};

STSPIN240_250* m_motorBrdg[2] = { NULL, NULL};

/* Private function prototypes -----------------------------------------------*/
void SpinIRQHandler(void);
void STSpinErrorHandler(uint16_t error);
	
/* Private functions ---------------------------------------------------------*/

void STSpinInit()
{
	// Printing to the console. 
	PRINTF("Initializing STSpin Motor Controller... ");
	
	m_motorBrdg[0] = new STSPIN240_250(PA_6, PC_4, PB_10, PA_8, PB_0, PA_11, PA_0);		// PA_10 (EN), PC_7 (STBY), PB_10 (PHA), PA_8 (PHB), PB_4 (PWMA), PB_5 (PWMB), PA_0 (REF)
	//m_motorBrdg[1] = new STSPIN240_250(PC_5, PA_4, PC_3, PB_2, PB_1, PA_1, PA_0);		// PA_1 (EN), PA_4 (STBY), PA_5 (PHA), PB_1 (PHB), PB_6 (PWMA), PB_0 (PWMB), PA_6 (REF)
	
	for (int i = 0; i < MAX_NUMBER_OF_DEVICES; ++i)
	{
		if (m_motorBrdg[i]->Init(&initDeviceParameters) != COMPONENT_OK) exit(EXIT_FAILURE);
	
		// Set dual bridge enabled as two motors are used
		m_motorBrdg[i]->SetDualFullBridgeConfig(1);
    
		// Attaching and enabling an interrupt handler.
		m_motorBrdg[i]->AttachFlagIRQ(&SpinIRQHandler);
		m_motorBrdg[i]->EnableFlagIRQ();
    
		// Attaching an error handler
		m_motorBrdg[i]->AttachErrorHandler(&STSpinErrorHandler);
    
		// Set PWM Frequency of Ref to 15000 Hz 
		m_motorBrdg[i]->SetRefPwmFreq(0, 15000); 
    
		// Set PWM duty cycle of Ref to 60% 
		m_motorBrdg[i]->SetRefPwmDc(0, 60);
    
		// Set PWM Frequency of bridge A inputs to 10000 Hz 
		m_motorBrdg[i]->SetBridgeInputPwmFreq(0, 10000); 
    
		// Set PWM Frequency of bridge B inputs to 10000 Hz 
		m_motorBrdg[i]->SetBridgeInputPwmFreq(1, 10000); 
	}
	
	//PRINTF("complete.\r\n");
}

void STSpinDestroy()
{
	for (int i = 0; i < MAX_NUMBER_OF_DEVICES; ++i)
	{
		if (m_motorBrdg[i])
		{
			delete m_motorBrdg[i];
			m_motorBrdg[i] = NULL;
		}
	}
}

void STSpinSetMotor(uint8_t deviceIndx, uint8_t motorIndx, float dutyCycle, direction_t dir)
{
	if (deviceIndx >= MAX_NUMBER_OF_DEVICES)
		return;
	
	if (dutyCycle == 0)
	{
		m_motorBrdg[deviceIndx]->HardHiZ(motorIndx);
	}
	else
	{
		m_motorBrdg[deviceIndx]->SetSpeed(motorIndx, dutyCycle); 
		m_motorBrdg[deviceIndx]->Run(motorIndx, dir);
	}
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void SpinIRQHandler(void)
{
	//PRINTF("\r\n\r\n");
	//PRINTF("*** WARNING: \"STSpin FLAG\" interrupt triggered. ***\r\n");

	for (int i = 0; i < MAX_NUMBER_OF_DEVICES; ++i)
	{
		for (int j = 0; j < MAX_NUMBER_OF_BRUSH_DC_MOTORS; ++j)
		{
			// Get the state of this bridge
			uint16_t bridgeState  = m_motorBrdg[i]->GetBridgeStatus(j);
			if (bridgeState == 0) 
			{
				if (m_motorBrdg[i]->GetDeviceState(j) != INACTIVE)
				{
					/* Bridges were disabled due to overcurrent or over temperature */
					/* When  motor was running */
					STSpinErrorHandler(0XBAD0);
				}
			}
		}
	}
	
}

void STSpinErrorHandler(uint16_t error)
{
  /* Printing to the console. */
	PRINTF("STSpin Error %d detected\r\n\n", error);
  
	/* Infinite loop */
	while (1)
	{
	}    
}