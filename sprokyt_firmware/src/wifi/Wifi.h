#ifndef _WIFI_H_
#define _WIFI_H_

#include "ESP8266.h"
#include "Endpoint.h"
#include "TCPSocketServer.h"
#include <string>

extern "C"
{
	void print_buffer_ESP8266();
}

class Wifi
{
public:
	static Wifi* Instance();
	void Init();
	void Update();
	void Reset();
	void ParseWifiMessage(const char* rxStr);
	const char* ParseInstruction(const char* rxStr);
	static void RxCallback(const char* rxStr);
	
private:
	Wifi();
	~Wifi();
	Wifi& operator=(const Wifi&);
	
	void PrintBufferESP8266();
	
	static Wifi* m_pInstance;
	TCPSocketServer m_server;
	string m_rxStr;
	int m_numConnections;
};

#include "UDPSocket.h"

#endif // _WIFI_H_
