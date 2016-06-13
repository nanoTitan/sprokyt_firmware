/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BLE_H_
#define __BLE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "hci.h"
	 
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
	 
void Init_BLE();
void Update_BLE();
	 
/** @addtogroup SPROKYT_BLE_Exported_Functions
*  @{
*/
void HCI_Event_CB(void *pckt);
	 
#ifdef __cplusplus
}
#endif

#endif // __BLE_H_