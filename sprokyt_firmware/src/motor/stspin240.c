#include "stspin240.h"
#include "motorcontrol.h"
#include "error.h"
#include "debug.h"
#include "stspin240_250.h"

#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_nucleo_ihm12a1.h"
#endif
#ifdef USE_STM32F0XX_NUCLEO
#include "stm32f0xx_nucleo_ihm12a1.h"
#endif
#ifdef USE_STM32L0XX_NUCLEO
#include "stm32l0xx_nucleo_ihm12a1.h"
#endif
#ifdef USE_STM32F3XX_NUCLEO
#include "stm32f3xx_nucleo_ihm12a1.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MAX_STEPS (12)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile uint8_t gButtonPressed = FALSE;
static volatile uint8_t gStep = MAX_STEPS;

Stspin240_250_Init_t gStspin240_250InitParams =
{
	50000, // Frequency of PWM of Input Bridge A in Hz up to 100000Hz,	default was 20000
	50000, // Frequency of PWM of Input Bridge B in Hz up to 100000Hz,	default was 20000
	50000,                // Frequency of PWM used for Ref pin in Hz up to 100000Hz,	default was 20000
	50,                   //Duty cycle of PWM used for Ref pin (from 0 to 100)
	TRUE                  // Dual Bridge configuration  (FALSE for mono, TRUE for dual brush dc)
};

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
void ButtonHandler(void);
/* Private functions ---------------------------------------------------------*/


void STSpin240_Init()
{
	uint8_t result = BSP_MotorControl_SetNbDevices(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240, 1);
	if (result != 1)
		CError_Handler();
	
	//BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240, &gStspin240_250InitParams);		// Initialize STSpin from Stspin240_250_Init_t struct
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_STSPIN240, NULL);							// Initialize STSpin from tspin240_250_target_config.h
	
	/* Set dual bridge enabled as two motors are used*/
	BSP_MotorControl_SetDualFullBridgeConfig(0);
  
	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	  /* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(CError_Handler_1);
 
    /* Set PWM Frequency of Ref to 15000 Hz */ 
	BSP_MotorControl_SetRefFreq(0, 50000); 

	  /* Set PWM duty cycle of Ref to 60% */ 
	BSP_MotorControl_SetRefDc(0, 50); 
  
	/* Set PWM Frequency of bridge A inputs to 10000 Hz */ 
	BSP_MotorControl_SetBridgeInputPwmFreq(0, 50000); 
  
	/* Set PWM Frequency of bridge B inputs to 10000 Hz */ 
	/* On X-NUCLEO-IHM12A1 expansion board PWM_A and PWM_B shares the same */
	/* timer, so frequency must be the same */
	BSP_MotorControl_SetBridgeInputPwmFreq(1, 50000); 
	
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
  /* Code to be customised */
  /************************/
  /* Get the state of bridge A */
	uint16_t bridgeState  = BSP_MotorControl_CmdGetStatus(0);
  
	if (bridgeState == 0) 
	{
		if (BSP_MotorControl_GetDeviceState(0) != INACTIVE)
		{
		  /* Bridges were disabled due to overcurrent or over temperature */
		  /* When  motor was running */
			CError_Handler_1(0XBAD0);
		}
	}
}