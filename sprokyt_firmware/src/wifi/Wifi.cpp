#include "Wifi.h"
#include "debug.h"

char DEFAULT_WIFI_SSID[] = { 'm', 'y', 'C', 'o', 'o', 'l', 'W', 'i', 'F', 'i' };
char DEFAULT_WIFI_PWRD[] = { '1', '2', '3', '4', '5', '6', '7', '8' };

Wifi* Wifi::m_pInstance = NULL;

// For monitoring data from ESP8266
Timeout timer_buffer_debug;
CircBuffer<char> buffer_ESP8266_recv(1024);
RawSerial pc(USBTX, USBRX);
ESP8266 esp(D8, D2, D9, "NETGEAR35", "yellowquail877", 115200);		// tx, rx - D1, D0, D2   - 115200

void print_buffer_ESP8266()
{
	char c = 0;
	while (buffer_ESP8266_recv.available())
	{
		buffer_ESP8266_recv.dequeue(&c);
		pc.putc(c);
	}
	timer_buffer_debug.attach(&print_buffer_ESP8266, 0.1);
}

Wifi::Wifi()
{
	pc.baud(9600);	
	print_buffer_ESP8266();
}

Wifi::~Wifi()
{
	esp.close();
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
	esp.reset();	
	//esp.startServerWithAP(DEFAULT_WIFI_SSID, DEFAULT_WIFI_PWRD, 7, 4, 700);
	
	//esp.join();
	
	bool success = esp.connect();
	if (!success)  
	{
		PRINTF("wifi.connect error\r\n");
		return;
	}
	
	success = esp.startTCPServer(1001);
	if (!success)
	{
		PRINTF("server.bind error\r\n");
		return;
	}
	
}

void Wifi::Update()
{
}

void Wifi::Reset()
{
	ESP8266::getInstance()->reset();
}