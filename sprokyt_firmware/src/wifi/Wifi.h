#ifndef _WIFI_H_
#define _WIFI_H_

#include "ESP8266.h"
#include "Endpoint.h"
#include "TCPSocketServer.h"
#include "rtos.h"
#include <string>
#include <vector>

#define PRINT_WIFI

class Wifi
{
public:
	static Wifi* Instance();
	void Init();
	void Update();
	void Reset();
	bool IsConnected() { return m_numConnections != 0; }
	
	static void RxCallback(void const *args);
	
private:
	Wifi();
	~Wifi();
	Wifi& operator=(const Wifi&);
	
	bool ParseIPD();
	void ParseInstructions();
	void HandleRx();
	void PrintBufferESP8266();
	void DequeueRxBuff();
	
	static Wifi* m_pInstance;
	TCPSocketServer m_server;
	Timeout m_rxTimer;
	string m_rxStr;	
	std::vector<uint8_t> m_rxData;
	Thread m_thread;
	int m_numConnections;	
	bool m_rxTimerAttached;
	bool m_isInitialized;
};

#include "UDPSocket.h"

#endif // _WIFI_H_
