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
	wifi_state_ready,
	wifi_state_idle,
	wifi_state_connected,
	wifi_state_connecting,
	wifi_state_disconnected,
	wifi_state_socket,
	wifi_state_write,
	wifi_state_error,
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
	WiFi_Status_t wifi_get_AP_settings(void);
	
	static SWPF01SA* m_pInstance;
	char m_print_msg_buff[512];
	
	wifi_state_t m_wifi_state;
	wifi_config m_config;
	uint8_t m_console_input[1];
	uint8_t m_console_count = 0;
	char m_console_ssid[40];
	wifi_bool m_set_AP_config = WIFI_FALSE;

	char* m_ssid;
	char* m_seckey;
	uint8_t m_channel_num = 6;
	WiFi_Priv_Mode m_mode = WPA_Personal;    
	uint16_t m_len;
	
	WiFi_Status_t m_status = WiFi_MODULE_SUCCESS;
	char m_protocol;
	uint32_t m_portnumber = 32000;
};

/** @addtogroup SPROKYT_BLE_Exported_Functions
*  @{
*/

