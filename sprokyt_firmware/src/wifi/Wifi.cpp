#include "Wifi.h"

char DEFAULT_WIFI_SSID[] = { 'm', 'y', 'C', 'o', 'o', 'l', 'W', 'i', 'F', 'i' };
char DEFAULT_WIFI_PWRD[] = { '1', '2', '3', '4', '5', '6', '7', '8' };

Wifi* Wifi::m_pInstance = NULL;

Wifi::Wifi()
{
}

Wifi* Wifi::Instance()
{
	if (!m_pInstance)
	{
		static Wifi wifi;
		m_pInstance = &wifi;
	}
	
	return m_pInstance;
}

void Wifi::Init()
{
	ESP8266 esp(D8, D2, D9, "NETGEAR35", "yellowquail877", 115200);		// tx, rx - D1, D0, D2   - 115200
	esp.reset();	
	esp.startSoftAP(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PWRD, 7, 4);
}

void Wifi::Update()
{
    // TODO
}

void Wifi::Reset()
{
	ESP8266::getInstance()->reset();
}