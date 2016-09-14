#ifndef _WIFI_H_
#define _WIFI_H_

#include "ESP8266.h"
#include "Endpoint.h"
#include "TCPSocketServer.h"

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
	
private:
	Wifi();
	~Wifi();
	Wifi& operator=(const Wifi&);
	
	static Wifi* m_pInstance;
	TCPSocketServer m_server;
};

#include "UDPSocket.h"

#endif // _WIFI_H_
