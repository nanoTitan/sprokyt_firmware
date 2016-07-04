/* Define to prevent recursive inclusion -------------------------------------*/
#pragma once

extern "C" {
#include "hci.h"
}
	
#include <stdint.h>
	 
/*
*         Main function to show how to use the BlueNRG Bluetooth Low Energy
*         expansion board to send data from a Nucleo board to a smartphone
*         with the support BLE and the "BlueNRG" app freely available on both
*         GooglePlay and iTunes.
*         The URL to the iTunes for the "BlueNRG" app is
*         http://itunes.apple.com/app/bluenrg/id705873549?uo=5
*         The URL to the GooglePlay is
*         https://play.google.com/store/apps/details?id=com.st.bluenrg
*         The source code of the "BlueNRG" app, both for iOS and Android, is
*         freely downloadable from the developer website at
*         http://software.g-maps.it/
*         The board will act as Server-Peripheral.
*
*         After connection has been established:
*          - by pressing the USER button on the board, the cube showed by
*            the app on the smartphone will rotate.
*          
*         The communication is done using a vendor specific profile.
*/
	 
/* Exported defines ----------------------------------------------------------*/   
#define IDB04A1 0
#define IDB05A1 1
#define BDADDR_SIZE 6
#define JOYSTICK_COUNT 4

/** 
* @brief Structure containing acceleration value (in mg) of each axis.
*/
typedef struct 
{
	int32_t AXIS_X;
	int32_t AXIS_Y;
	int32_t AXIS_Z;
} AxesRaw_t;
	 
/** 
* @brief Structure containing acceleration value (in mg) of each axis.
*/
typedef struct 
{
	uint8_t x;
	uint8_t y;
} Joystick_t;

extern "C"
{
	void HCI_Event_CB(void *pckt);
}

class BLE
{
public:
	static void InitBLE();
	static void Update();
	static uint8_t GetExpansionBoard() { return bnrg_expansion_board; }
	static void Read_Request_CB(uint16_t handle);
	static void GAPDisconnectionCompleteCB(void);
	static void GAPConnectionCompleteCB(uint8_t addr[6], uint16_t handle);
	static void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data);
	
private:
	static tBleStatus AddAccService(void);
	static tBleStatus AddLEDService(void);
	static tBleStatus AddInputService(void);
	static tBleStatus AddInstructionService(void);
	static tBleStatus AddControlSensorService(void);
	static void User_Process();
	static void setBLEConnectable(void);
	static tBleStatus Free_Fall_Notify(void);
	static tBleStatus AccUpdate(AxesRaw_t *data);
	
	static uint8_t SERVER_BDADDR[];// = { 0x12, 0x34, 0x00, 0xE1, 0x80, 0x03 };	
	static uint8_t bdaddr[BDADDR_SIZE];
	static uint8_t bnrg_expansion_board; /* at startup, suppose the X-NUCLEO-IDB04A1 is used */	
	static AxesRaw_t axes_data[];
	static volatile uint8_t do_set_connectable;
	static volatile uint16_t service_connection_handle;
	static volatile uint8_t is_notification_enabled;
	static volatile int connected;
	static uint16_t TXCharHandle;
	static uint16_t RXCharHandle;
	static uint16_t accServHandle;
	static uint16_t freeFallCharHandle;
	static uint16_t accCharHandle;
	static uint16_t envSensServHandle;
	static uint16_t tempCharHandle;
	static uint16_t pressCharHandle;
	static uint16_t humidityCharHandle;
	static uint16_t ledServHandle;
	static uint16_t ledButtonCharHandle;
	static uint16_t inputServHandle;
	static uint16_t inputButtonCharHandle;
	static uint16_t instructionServHandle;
	static uint16_t instructionButtonCharHandle;
};

/** @addtogroup SPROKYT_BLE_Exported_Functions
*  @{
*/

