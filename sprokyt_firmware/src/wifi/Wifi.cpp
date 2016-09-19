#include "Wifi.h"
#include "control_manager.h"
#include <algorithm>
#include "debug.h"

string DEFAULT_WIFI_SSID = "myWifi2";
string DEFAULT_WIFI_PWRD = "11111111";

Wifi* Wifi::m_pInstance = NULL;

// For monitoring data from ESP8266
Timeout timer_buffer_debug;
CircBuffer<char> buffer_ESP8266_recv(2048);
RawSerial pc(USBTX, USBRX);
ESP8266 esp(D8, D2, D9, "NETGEAR35", "yellowquail877", 115200);		// tx, rx - D1, D0, D2   - 115200

Wifi::Wifi()
:m_numConnections(0),
 m_rxTimerAttached(false),
 m_isInitialized(false)
{
	pc.baud(9600);	
	HandleRx();
	esp.setReceiveCallback(RxCallback);
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
	
	// Clear vector
	m_isInitialized = true;
	
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
}

void Wifi::Update()
{	
}

void Wifi::Reset()
{
	ESP8266::getInstance()->reset();
}

bool Wifi::ParseIPD()
{
	// Remove "+IPD,x,
	char c;
	char szBuff[4] = {0, 0, 0, 0};
	int msgSz = 0;
	int bytesRead = 7;
	
	// Early check to see if we have ':' deliminator
	if ( !buffer_ESP8266_recv.isValid(bytesRead + 3) )
		return false;
	
	int indx = 0;
	for(int i = 4; i > 0; --i)
	{
		c = buffer_ESP8266_recv.getAt(bytesRead);
		if (c == ':')
		{
			msgSz = strtol(szBuff, NULL, 0);
			++bytesRead;
			break;
		}
		else
		{
			szBuff[indx] = c;
			++indx;
			++bytesRead;
		}
	}
	
	if ( !buffer_ESP8266_recv.isValid(bytesRead + msgSz - 1) )
		return false;
	
	for (int i = msgSz; i > 0; --i)
	{
		c = buffer_ESP8266_recv.getAt(bytesRead++);
		m_rxData.push_back(c);
	}
	
	for (int i = bytesRead; i > 0; --i)
		DequeueRxBuff();
	
	return true;
}

void Wifi::ParseInstructions()
{
	int indx = 0;
	uint8_t data[16];
	bool parsedInstr = false;
	while (indx < m_rxData.size())
	{
		int instrSz = m_rxData[indx++];
		if (m_rxData.size() - 1 < instrSz)
			break;
		
		for (int i = 0; i < instrSz; ++i)
		{
			data[i] = m_rxData[indx++];
		}
		
		ControlMgr_parseInstruction(instrSz, data);
		parsedInstr = true;
	}
	
	if (parsedInstr)
		m_rxData.erase(m_rxData.begin(), m_rxData.begin() + indx);
}

void Wifi::RxCallback()
{
	//Instance()->m_rxTimer.attach(Instance(), &Wifi::HandleRx, 0.1);
}

void Wifi::HandleRx()
{	
	float resetTime = 0.1f;
	if (!m_isInitialized || buffer_ESP8266_recv.isEmpty())
	{
		PrintBufferESP8266();
		timer_buffer_debug.attach(this, &Wifi::HandleRx, resetTime);
		return;
	}
	
	char c;
	const char* term = NULL;
	uint32_t indx = 0;
	bool partialInstrFound = false;
	
	while (true)
	{		
		// "Connect"
		term = "CONNECT";
		indx = buffer_ESP8266_recv.find(term, strlen(term));
		if (indx != -1)
		{
			++m_numConnections;
			
			uint32_t length = buffer_ESP8266_recv.getLength(indx + strlen(term)) ;
			string s;
			for (uint32_t i = 0; i < length; ++i)
			{
				buffer_ESP8266_recv.dequeue(&c);
				s.push_back(c);
			}
			
			PRINTF(s.c_str()); PRINTF("\r\n");			
			continue;
		}
		
		// "Disconnect"
		term = "CLOSED";
		indx = buffer_ESP8266_recv.find(term, strlen(term));
		if (indx != -1)
		{
			--m_numConnections;
			if (m_numConnections == 0)
			{
				m_rxData.erase(m_rxData.begin(), m_rxData.end());
				ControlMgr_connectionLost();
			}
			
			if (m_numConnections < 0)
				m_numConnections = 0;
			
			uint32_t length = buffer_ESP8266_recv.getLength(indx + 7);
			string s;
			for (uint32_t i = 0; i < length; ++i)
			{
				buffer_ESP8266_recv.dequeue(&c);
				s.push_back(c);
			}
			
			PRINTF(s.c_str()); PRINTF("\r\n");			
			continue;
		}
		
		// "+IPD"
		if (m_numConnections > 0)
		{			
			term = "+IPD";
			indx = buffer_ESP8266_recv.find(term, strlen(term));
			if (indx != -1)
			{
				uint32_t length = buffer_ESP8266_recv.getLength(indx);
				for (uint32_t i = length - 1; i > 0; --i)
					DequeueRxBuff();
				
				if (ParseIPD())
				{
					ParseInstructions();
					continue;
				}
				
				partialInstrFound = true;
			}
		}
		
		if (!partialInstrFound)
		{			
			PrintBufferESP8266();			
		}
		
		break;
		
		// "Carriage Return"
//		while (true)
//		{
//			c = '\r';
//			uint32_t indx = buffer_ESP8266_recv.find(c);
//			if (indx == -1)
//				break;
//			
//			uint32_t length = buffer_ESP8266_recv.getLength(indx);
//			for (uint32_t i = length; i > 0; --i)
//				DequeueRxBuff();
//		}
//		
//		// Newline
//		while (true)
//		{
//			c = '\n';
//			uint32_t indx = buffer_ESP8266_recv.find(c);
//			if (indx == -1)
//				break;
//			
//			uint32_t length = buffer_ESP8266_recv.getLength(indx);
//			for (uint32_t i = length; i > 0; --i)
//				DequeueRxBuff();
//		}
//		
//		// We got to the end, so leave
//		break;
	}
	
	timer_buffer_debug.attach(this, &Wifi::HandleRx, resetTime);
}

void Wifi::PrintBufferESP8266()
{
	char c;
	while (buffer_ESP8266_recv.available())
	{		
		buffer_ESP8266_recv.dequeue(&c);
		pc.putc(c);
	}
	
	//timer_buffer_debug.attach(this, &Wifi::PrintBufferESP8266, 0.1);
}

void Wifi::DequeueRxBuff()
{
	if (buffer_ESP8266_recv.available())
	{
		char c;
		buffer_ESP8266_recv.dequeue(&c);
		pc.putc(c);
	}
}