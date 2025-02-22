#include "SWPF01SA.h"
#include "wifi_globals.h"
#include "stm32_spwf_wifi.h"
#include "debug.h"
#include "error.h"

extern "C" {
}

/* Private  -----------------------------------------------*/
extern UART_HandleTypeDef UartHandle, UartMsgHandle;
char m_echo[512];

SWPF01SA* SWPF01SA::m_pInstance = NULL;

SWPF01SA::SWPF01SA()
:m_console_count(0),
m_set_AP_config(WIFI_FALSE),
m_channel_num(6),
m_mode(WPA_Personal),
m_len(0),	
m_status(WiFi_MODULE_SUCCESS),
m_portnumber(32000)
{
	m_pInstance = NULL;
	m_ssid = "SWPF_AS01";
	m_seckey = "12341234";
	m_protocol = 't';
	
	SetState(wifi_state_idle);
}

SWPF01SA::~SWPF01SA()
{
	
}

SWPF01SA* SWPF01SA::Instance()
{
	if (!m_pInstance)
	{
		static SWPF01SA wifi;
		m_pInstance = &wifi;
	}
	
	return m_pInstance;
}

bool SWPF01SA::InitWifi()
{
	WiFiTimerInit();
	WiFiPushTimerInit();
	
	UART_Configuration(115200); 
#ifdef USART_PRINT_MSG
	UART_Msg_Gpio_Init();
	USART_PRINT_MSG_Configuration(115200);
#endif  
	
	m_config.power = wifi_active;
	m_config.power_level = high;
	m_config.dhcp = on;//use DHCP IP address

	SetState(wifi_state_idle);

	PRINTF("\nInitializing WiFi module...");

	  /* Init the wi-fi module */  
	m_status = wifi_init(&m_config);
	if (m_status != WiFi_MODULE_SUCCESS)
	{
		PRINTF("Error in Config");
		return false;
	}
	
	return true;
}

void SWPF01SA::Update()
{
	switch (m_wifi_state) 
	{
	case wifi_state_reset:
		break;

	case wifi_state_ready:
		PRINTF(" >>setting up miniAP mode...\n");
        
		if (m_set_AP_config)
			wifi_ap_start((uint8_t *)m_console_ssid, m_channel_num);
		else
			wifi_ap_start((uint8_t *)m_ssid, m_channel_num);

			//        wifi_connect(console_ssid,seckey, mode);

		SetState(wifi_state_idle);
		break;

	case wifi_state_connected:
		PRINTF(" >>connected...\n");

		SetState(wifi_state_socket);
		break;

	case wifi_state_disconnected:
		SetState(wifi_state_reset);
		break;

	case wifi_state_socket:
		PRINTF(" >>WiFi_RW_Data\n");

		/* Read Write Socket data */        
		m_status = wifi_socket_server_open(m_portnumber, &m_protocol);

		if (m_status == WiFi_MODULE_SUCCESS)
		{
			PRINTF(" >>Server Socket Open OK \n");          
		}
		SetState(wifi_state_idle);

		break;

	case wifi_state_write:
		PRINTF(" >>Writing data to client\n");

		m_len = strlen(m_echo);
		/* Read Write Socket data */        
		m_status = wifi_socket_server_write(m_len, m_echo);

		if (m_status == WiFi_MODULE_SUCCESS)
		{
			PRINTF("Server Socket Write OK");
		}
		SetState(wifi_state_idle);

		break;

	case wifi_state_idle:
		PRINTF(".");
		fflush(stdout);
		HAL_Delay(500);
		break;

	default:
		break;
	}
}

void SWPF01SA::SetState(wifi_state_t state)
{
	if (state < wifi_undefine_state)
		m_wifi_state = state;
	else
		PRINTF("Undefined wifi state trying to be set: %d", state);
}

void SWPF01SA::WiFiTimerInit()
{
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  
	/* Set TIMx instance */
	TimHandle.Instance = TIM3;

	/* Initialize TIMx peripheral as follows:
	+ Period = 10000 - 1
	+ Prescaler = (SystemCoreClock/10000) - 1
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
#if defined (USE_STM32L0XX_NUCLEO) || (USE_STM32F4XX_NUCLEO) || (USE_STM32L4XX_NUCLEO)
	TimHandle.Init.Period            = 100 - 1;
#endif
#if defined (USE_STM32F1xx_NUCLEO) 
	TimHandle.Init.Period            = 100 - 1;
#endif  
	TimHandle.Init.Prescaler         = uwPrescalerValue;
	TimHandle.Init.ClockDivision     = 0;
	TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
#ifdef USE_STM32F1xx_NUCLEO
	TimHandle.Init.RepetitionCounter = 0;
#endif 

	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
	{
	  /* Initialization Error */
		CError_Handler(); 
	}
}

void SWPF01SA::WiFiPushTimerInit()
{
	/* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
	uint32_t uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
  
	/* Set TIMx instance */
	PushTimHandle.Instance = TIM4;

	  /* Initialize TIMx peripheral as follows:
	       + Period = 10000 - 1
	       + Prescaler = (SystemCoreClock/10000) - 1
	       + ClockDivision = 0
	       + Counter direction = Up
	     */
	PushTimHandle.Init.Period            = 10 - 1;//10000
	PushTimHandle.Init.Prescaler         = uwPrescalerValue;
	PushTimHandle.Init.ClockDivision     = 0;
	PushTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
#ifdef USE_STM32F1xx_NUCLEO
	PushTimHandle.Init.RepetitionCounter = 0;
#endif 

	if (HAL_TIM_Base_Init(&PushTimHandle) != HAL_OK)
	{
	  /* Initialization Error */
		CError_Handler(); 
	}
}

WiFi_Status_t SWPF01SA::wifi_get_AP_settings(void)
{
	WiFi_Status_t status = WiFi_MODULE_SUCCESS;
	
	PRINTF("*******************************************************\n");
	PRINTF("                                                      *\n");
	PRINTF(" X-CUBE-WIFI1 Expansion Software V2.1.0               *\n");
	PRINTF(" X-NUCLEO-IDW01M1 Wi-Fi Mini-AP Configuration.        *\n");
	PRINTF(" Server-Socket Example                                *\n");
	PRINTF("                                                      *\n");
	PRINTF("*******************************************************\n");
	PRINTF("Do you want to setup SSID?(y/n):");
	fflush(stdout);
	scanf("%s", m_console_input);
  
	//HAL_UART_Receive(&UartMsgHandle, (uint8_t *)console_input, 1, 100000);
	if (m_console_input[0] == 'y') 
	{
		m_set_AP_config = WIFI_TRUE;  
		PRINTF("Enter the SSID for mini-AP:");
		fflush(stdout);

		m_console_count = 0;
		m_console_count = scanf("%s", m_console_ssid);
		PRINTF("");

		if (m_console_count == 39) 
		{
			PRINTF("Exceeded number of ssid characters permitted");
			return WiFi_NOT_SUPPORTED;
		}

	}
	else 
	{
		PRINTF("Module will connect with default settings.\n");
		memcpy(m_console_ssid, (const char*)m_ssid, strlen((char*)m_ssid));             
	}

	PRINTF("*************************************************************\n");
	PRINTF("* Configuration Complete                                     \n");
	PRINTF("* Please make sure a server is listening at given hostname   \n");
	PRINTF("*************************************************************\n");

	return status;	
}

void ind_wifi_socket_data_received(uint8_t socket_id, uint8_t * data_ptr, uint32_t message_size, uint32_t chunk_size)
{
	PRINTF("Data Receive Callback...\n");
	memcpy(m_echo, data_ptr, 50);
	PRINTF((const char*)m_echo);
	PRINTF("socket ID: %d\n", socket_id);
	PRINTF("msg size: %lu\n", (unsigned long)message_size);
	PRINTF("chunk size: %lu\n", (unsigned long)chunk_size);
	fflush(stdout);
	//wifi_state = wifi_state_write;
}

void ind_wifi_on()
{
	SWPF01SA::Instance()->SetState(wifi_state_ready);
}

void ind_wifi_connected()
{
	SWPF01SA::Instance()->SetState(wifi_state_connected);
}

void ind_socket_server_client_joined(void)
{
	PRINTF("User callback: Client joined...\n");
	fflush(stdout);
}

void ind_socket_server_client_left(void)
{
	PRINTF("User callback: Client left...\n");
	fflush(stdout);
}

void ind_wifi_ap_client_joined(uint8_t * client_mac_address)
{
	PRINTF(">>client joined callback...\n");
	PRINTF((const char*)client_mac_address);
	fflush(stdout);  
}

void ind_wifi_ap_client_left(uint8_t * client_mac_address)
{
	PRINTF(">>client left callback...\n");
	PRINTF((const char*)client_mac_address);
	fflush(stdout);  
}