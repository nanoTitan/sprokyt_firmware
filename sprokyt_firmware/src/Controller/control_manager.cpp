#include "control_manager.h"
#include "PID.h"
#include "Wifi.h"
#include "user_control.h"
#include "flight_control.h"
#include "esc_programmer.h"
#include "debug.h"

/* Private variables ---------------------------------------------------------*/
enum CONTROL_STATE m_controlState = CONTROL_STATE_IDLE;
enum CONTROLLER_TYPE m_currControllerType = CONTROLLER_USER;
uint32_t m_lastPing = 0;
uint32_t m_vbat = 0;
ADC_HandleTypeDef m_hADC;
DMA_HandleTypeDef  m_hDMA;

/* Private function prototypes -----------------------------------------------*/
static void ControlMgr_setInstruction(uint8_t instruction, uint8_t value);
static void ResetPing();
static void ControlMgr_ConfigADC();

extern "C"
{
	void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
	{
		uint32_t adcVal = HAL_ADC_GetValue(AdcHandle);
		m_vbat = (adcVal * ADC_VBAT_MULTI) * ADC_SUPPLY_VOLTAGE / 0xFFF;
	}
	
	void ADC_IRQHandler()
	{
		HAL_ADC_IRQHandler(&m_hADC);
	}
}

CONTROL_STATE ControlMgr_getState()
{
	return m_controlState;
}

void ControlMgr_setState(CONTROL_STATE state)
{
	m_controlState = state;
}

void ControlMgr_setType(CONTROLLER_TYPE ctrlType)
{
	m_currControllerType = ctrlType;
	PRINTF("Setting Control State: %u\r\n", ctrlType);
}

void ControlMgr_init()
{
	ControlMgr_ConfigADC();
	FlightControl_init();
	UserControl_init();
}

void ControlMgr_ConfigureDMA()
{
	__DMA2_CLK_ENABLE(); 
	m_hDMA.Instance = DMA2_Stream4;
  
	m_hDMA.Init.Channel  = DMA_CHANNEL_0;
	m_hDMA.Init.Direction = DMA_PERIPH_TO_MEMORY;
	m_hDMA.Init.PeriphInc = DMA_PINC_DISABLE;
	m_hDMA.Init.MemInc = DMA_MINC_ENABLE;
	m_hDMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	m_hDMA.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	m_hDMA.Init.Mode = DMA_CIRCULAR;
	m_hDMA.Init.Priority = DMA_PRIORITY_HIGH;
	m_hDMA.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         
	m_hDMA.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
	m_hDMA.Init.MemBurst = DMA_MBURST_SINGLE;
	m_hDMA.Init.PeriphBurst = DMA_PBURST_SINGLE; 
    
	HAL_DMA_Init(&m_hDMA);
    
	__HAL_LINKDMA(&m_hADC, DMA_Handle, m_hDMA);
 
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 0, 0);   
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void ControlMgr_ConfigADC()
{	
	__ADC1_CLK_ENABLE();
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	ADC_ChannelConfTypeDef sConfig = { 0 };

	m_hADC.Instance = ADC1;
	
	m_hADC.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	m_hADC.Init.Resolution = ADC_RESOLUTION_12B;
	m_hADC.Init.ScanConvMode = DISABLE;
	m_hADC.Init.ContinuousConvMode = ENABLE;
	m_hADC.Init.DiscontinuousConvMode = DISABLE;
	m_hADC.Init.NbrOfDiscConversion = 0;
	m_hADC.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	m_hADC.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	m_hADC.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	m_hADC.Init.NbrOfConversion = 1;
	m_hADC.Init.DMAContinuousRequests = ENABLE;
	m_hADC.Init.EOCSelection = DISABLE;
	
	HAL_ADC_Init(&m_hADC);
	
    // Configure ADC channel
	sConfig.Channel      = ADC_CHANNEL_VBAT;
	sConfig.Rank         = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	sConfig.Offset       = 0;

	HAL_ADC_ConfigChannel(&m_hADC, &sConfig);
	HAL_ADC_Start_IT(&m_hADC);
}

uint32_t ControlMgr_Get_Battery()
{
	return m_vbat;
}

void ControlMgr_update()
{
//	if (HAL_ADC_PollForConversion(&m_hADC, 1000000) == HAL_OK)
//	{
//		uint32_t adcVal = HAL_ADC_GetValue(&m_hADC);
//		m_vbat = (adcVal * ADC_VBAT_MULTI) * ADC_SUPPLY_VOLTAGE / 0xFFF;
//	}
	
	if (ControlMgr_getState() == CONTROL_STATE_CONNECTED)
	{
		uint32_t delta = HAL_GetTick() - m_lastPing;
		if (delta > WIFI_PING_TIMEOUT)							// Ping check
		{
			ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
		}
		else if (!Wifi::Instance()->IsConnected())
			ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
	}
	
	switch (m_currControllerType)
	{
	case CONTROLLER_USER:
		UserControl_update();
		break;
		
	case CONTROLLER_FLIGHT:
		FlightControl_update();
		
	case CONTROLLER_ROVER:
		break;
		
	case CONTROLLER_ROVER_BALANCE:
		break;
		
	case CONTROLLER_ESC_PROGRAMMER:
		break;
		
	default:
		break;
	}
}

void ControlMgr_setMotor(uint8_t motorIndex, uint8_t value, direction_t dir)
{
	if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_setMotor(motorIndex, value, dir);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if (m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{
		EscProgrammer_setMotor(motorIndex, value, dir);
	}
}

void ControlMgr_parseInstruction(uint8_t data_length, uint8_t *att_data)
{	
	if (data_length == 0)
		return;
	
	// Check for ping
	uint8_t instruction = att_data[0];
	if (instruction == INSTRUCTION_PING)
	{
		ResetPing();		
		return;
	}
	
	// Check for controller type change
	if (instruction == INSTRUCTION_CONTROL_TYPE)
	{
		if (data_length != 2)
		{
			PRINTF("Error: Instruction length is not 2\r\n");	
			return;
		}	
		
		uint8_t type = att_data[1];
		if (type < CONTROLLER_COUNT)
		{
			ControlMgr_setType((CONTROLLER_TYPE)type);			
		}	
		
		ResetPing();
		return;
	}
	
	// Update based on controller type
	if (m_currControllerType == CONTROLLER_USER)
	{
		UserControl_parseInstruction(data_length, att_data);
	}
	else if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_parseInstruction(data_length, att_data);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if (m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{
	}
	
	ResetPing();
}

void ResetPing()
{
	ControlMgr_setState(CONTROL_STATE_CONNECTED);
	PRINTF("dT: %lu\r\n", HAL_GetTick() - m_lastPing);	
	m_lastPing = HAL_GetTick();
}

void ControlMgr_setInstruction(uint8_t instruction, uint8_t value)
{
	if (m_currControllerType == CONTROLLER_FLIGHT)
	{
		FlightControl_setInstruction(instruction, value);
	}
	else if (m_currControllerType == CONTROLLER_ROVER)
	{
	}
	else if (m_currControllerType == CONTROLLER_ROVER_BALANCE)
	{
	}
	else if (m_currControllerType == CONTROLLER_ESC_PROGRAMMER)
	{		
		EscProgrammer_setInstruction(instruction, value);
	}
}
