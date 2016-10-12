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
ESP8266 esp(PA_9, PA_10, PC_10, "NETGEAR35", "yellowquail877", 115200);		// tx, rx - D1, D0, D2   - 115200

Wifi::Wifi()
:m_wifiCallbackTime(0.05),
 m_numConnections(0),
 m_doWifiTO(false),
 m_isInitialized(false)
 //,m_thread(Wifi::RxCallback)
{
	pc.baud(9600);
	m_wifiTO.attach(this, &Wifi::RxCallback, m_wifiCallbackTime);
		
	//HandleRx();
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
	PRINTF("Wifi - Initializing\r\n");
	
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
	if (m_doWifiTO)
	{
		m_doWifiTO = false;
		HandleRx();
	}
}

void Wifi::Reset()
{
	ESP8266::getInstance()->reset();
}

bool Wifi::ParseIPD()
{	
	// Remove "+IPD,x,
	char* c = NULL;
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
		if (!c)
			return false;
		
		if (*c == ':')
		{
			msgSz = strtol(szBuff, NULL, 0);
			++bytesRead;
			break;
		}
		else
		{
			szBuff[indx] = *c;
			++indx;
			++bytesRead;
		}
	}
	
	if ( !buffer_ESP8266_recv.isValid(bytesRead + msgSz - 1) )
		return false;
	
	bool plus = false;
	for (int i = msgSz; i > 0; --i)
	{
		c = buffer_ESP8266_recv.getAt(bytesRead++);
		m_rxData.push_back(*c);
		if (*c == '+')
			plus = true;
		else if (plus && *c == 'I')
		{
			PRINTF("Parsing too much!!!");
		}
		else
			plus = false;
	}
	
	for (int i = bytesRead; i > 0; --i)
		DequeueRxBuff();
	
	ParseInstructions();
	return true;
}

void Wifi::ParseInstructions()
{
	//PRINTF("ParseInstructions\r\n");
	
	int indx = 0;
	uint8_t data[16];
	bool parsedInstr = false;
	while (indx < m_rxData.size())
	{
		int instrSz = m_rxData[indx++];
		if (instrSz != 2)
		{
			PRINTF("Instr not size 3!!!\r\n");
		}
		
		if (m_rxData.size() - 1 < instrSz)
			break;
		
		for (int i = 0; i < instrSz; ++i)
		{
			data[i] = m_rxData[indx++];
			if(i > 15)
				PRINTF("data is too small!!!\r\n");
		}
		
		ControlMgr_parseInstruction(instrSz, data);
		parsedInstr = true;
	}
	
	if (parsedInstr)
	{		
		if (indx > m_rxData.size())
		{
			PRINTF("indx > size\r\n");
			indx = m_rxData.size();
		}
		
		//PRINTF("m_rxData erase: %d %d\r\n", m_rxData.size(), indx);
		m_rxData.erase(m_rxData.begin(), m_rxData.begin() + indx);
	}		
}

void Wifi::RxCallback()
{
	if (!m_isInitialized || buffer_ESP8266_recv.isEmpty())
	{
		if (!m_isInitialized)
			PrintBufferESP8266();
		
		m_wifiTO.detach();
		m_wifiTO.attach(this, &Wifi::RxCallback, m_wifiCallbackTime);
	}
	else
	{
		m_doWifiTO = true;
	}
}

void Wifi::HandleRx()
{	
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
			PRINTF("WiFi - connection created, connections: %d\r\n", m_numConnections);
			
			uint32_t length = buffer_ESP8266_recv.getLength(indx + strlen(term));
			for (uint32_t i = 0; i < length; ++i)
				DequeueRxBuff();
					
			continue;
		}
		
		// "Disconnect"
		term = "CLOSED";
		indx = buffer_ESP8266_recv.find(term, strlen(term));
		if (indx != -1)
		{
			--m_numConnections;
			PRINTF("WiFi - connection closed, connections: %d\r\n", m_numConnections);
			
			if (m_numConnections == 0)
			{
				m_rxData.erase(m_rxData.begin(), m_rxData.end());
				ControlMgr_setState(CONTROL_STATE_DISCONNECTED);
			}
			
			if (m_numConnections < 0)
				m_numConnections = 0;
			
			uint32_t length = buffer_ESP8266_recv.getLength(indx + 7);
			for (uint32_t i = 0; i < length; ++i)
				DequeueRxBuff();		
			
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
					continue;
				
				partialInstrFound = true;
			}
		}
		
		if (partialInstrFound && m_rxData.size() > 0)
		{	
			PRINTF("Wifi - partial Instr Found. Clearing rxData buffer!\r\n");
			m_rxData.erase(m_rxData.begin(), m_rxData.end());
		}
			
		PrintBufferESP8266();
		break;
	}
	
	m_wifiTO.detach();
	m_wifiTO.attach(this, &Wifi::RxCallback, m_wifiCallbackTime);
}

void Wifi::PrintBufferESP8266()
{
	char c;
	while (buffer_ESP8266_recv.available())
	{		
		buffer_ESP8266_recv.dequeue(&c);
		
#ifdef PRINT_WIFI
		pc.putc(c);
#endif // PRINT_WIFI
	}
}

void Wifi::DequeueRxBuff()
{
	if (buffer_ESP8266_recv.available())
	{
		char c;
		buffer_ESP8266_recv.dequeue(&c);

#ifdef PRINT_WIFI
		pc.putc(c);
#endif // PRINT_WIFI
	}
}