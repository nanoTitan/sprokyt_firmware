#ifndef _WIFI_H_
#define _WIFI_H_

#include "ESP8266.h"
#include "Endpoint.h"

class Wifi
{
public:
	Wifi* Instance();
	void Init();
	void Update();
	void Reset();
	
private:
	Wifi();
	~Wifi();
	Wifi& operator=(const Wifi&);
	
	static Wifi* m_pInstance;
};

#include "UDPSocket.h"

#endif // _WIFI_H_
