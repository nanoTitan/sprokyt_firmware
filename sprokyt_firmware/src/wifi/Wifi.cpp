#include "Wifi.h"
#include "control_manager.h"
#include <string>
#include "debug.h"

string DEFAULT_WIFI_SSID = "myWifi2";
string DEFAULT_WIFI_PWRD = "11111111";

Wifi* Wifi::m_pInstance = NULL;

// For monitoring data from ESP8266
Timeout timer_buffer_debug;
CircBuffer<char> buffer_ESP8266_recv(1024);
RawSerial pc(USBTX, USBRX);
ESP8266 esp(D8, D2, D9, "NETGEAR35", "yellowquail877", 115200);		// tx, rx - D1, D0, D2   - 115200

Wifi::Wifi()
:m_numConnections(0)
{
	pc.baud(9600);	
	PrintBufferESP8266();
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
	esp.startServerWithAP((char*)DEFAULT_WIFI_SSID.c_str(), (char*)DEFAULT_WIFI_PWRD.c_str(), 7, 4, 1001);
	
	/*
	// Test server connecting to wifi
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
	*/
	
	esp.setReceiveCallback(RxCallback);
}

void Wifi::Update()
{
	
}

void Wifi::Reset()
{
	ESP8266::getInstance()->reset();
}

void Wifi::ParseWifiMessage(const char* rxStr)
{	
	if (strstr(rxStr, "CONNECT") != 0)
	{
		++m_numConnections;
	}
	
	if (strstr(rxStr, "CLOSED") != 0)
	{
		--m_numConnections;
		if(m_numConnections == 0)
			ControlMgr_connectionLost();
	}
	
	if (m_numConnections <= 0)
	{
		m_numConnections = 0;
		return;
	}
	
	const char* curr = rxStr;
	while (curr)
		curr = ParseInstruction(curr);
}

const char* Wifi::ParseInstruction(const char* rxStr)
{	
	// Parse Instruction
	const char* curr = strstr(rxStr, "+IPD,");
	if(curr)
	{
		curr = curr + 7;	// Move past +IPD,X,
		char* sep = strchr(curr, ':');
		if (!sep || !(sep + 1))
			return NULL;
		
		char buff[8];
		memset(buff, 0, sizeof(buff));
		int size = sep - curr;
		memcpy(buff, curr, size);
		int len = atoi(buff);
		curr = sep + 1;
		size_t szBuff = 16;
		uint8_t data[szBuff];
		long l = 0;
		memset(data, 0, sizeof(data));
		for (int i = 0; i < len; ++i)
		{
			data[i] = (uint8_t)*curr;
			++curr;
		}
		
		ControlMgr_parseInstruction(len, data);
	}
	
	return curr;
}

void Wifi::RxCallback(const char* rxStr)
{
	Instance()->ParseWifiMessage(rxStr);
}

void Wifi::PrintBufferESP8266()
{
	char c;
	while (buffer_ESP8266_recv.available())
	{		
		buffer_ESP8266_recv.dequeue(&c);
		pc.putc(c);
		m_rxStr.push_back(c);
	}
	
	if (m_rxStr.find_last_of("\r\n\r\n") != string::npos)
	{
		ParseWifiMessage(m_rxStr.c_str());
		m_rxStr = "";
	}
	
	timer_buffer_debug.attach(this, &Wifi::PrintBufferESP8266, 0.1);
}