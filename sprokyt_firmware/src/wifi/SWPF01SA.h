/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

extern "C" {
}
	
#include "wifi_interface.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx.h"

typedef enum {
	wifi_state_reset        = 0,
	wifi_state_ready		= 1,
	wifi_state_idle			= 2,
	wifi_state_connected	= 3,
	wifi_state_connecting	= 4,
	wifi_state_disconnected	= 5,
	wifi_state_socket		= 6,
	wifi_state_write		= 7,
	wifi_state_error		= 8,
	wifi_undefine_state     = 0xFF,  
} wifi_state_t;


class SWPF01SA
{
public:
	static SWPF01SA* Instance();
	bool InitWifi();
	void Update();
	void SetState(wifi_state_t state);
	
private:
	SWPF01SA();
	~SWPF01SA();
	
	void WiFiTimerInit();
	void WiFiPushTimerInit();
	WiFi_Status_t wifi_get_AP_settings(void);
	
	static SWPF01SA* m_pInstance;
	char m_print_msg_buff[512];
	char m_console_ssid[40];
	
	wifi_state_t m_wifi_state;
	wifi_config m_config;
	uint8_t m_console_input[1];
	uint8_t m_console_count;	
	wifi_bool m_set_AP_config;

	char* m_ssid;
	char* m_seckey;
	uint8_t m_channel_num;
	WiFi_Priv_Mode m_mode;    
	uint16_t m_len;
	
	WiFi_Status_t m_status;
	uint8_t m_protocol;
	uint32_t m_portnumber;
};

/** @addtogroup SPROKYT_BLE_Exported_Functions
*  @{
*/

