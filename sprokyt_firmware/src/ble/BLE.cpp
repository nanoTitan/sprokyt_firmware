#include "BLE.h"
#include "cube_hal.h"
#include "hal_types.h"
#include "string.h"
#include "gp_timer.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"
#include "math.h"
#include "ControlManager.h"

extern "C" {
#include "hci_const.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "bluenrg_aci_const.h"  
#include "bluenrg_hal_aci.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"
}

#include <stdlib.h>


#define COPY_UUID_128_V2(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

#define COPY_ACC_SERVICE_UUID(uuid_struct)  COPY_UUID_128_V2(uuid_struct,0x02,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_FREE_FALL_UUID(uuid_struct)    COPY_UUID_128_V2(uuid_struct,0xe2,0x3e,0x78,0xa0, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_ACC_UUID(uuid_struct)          COPY_UUID_128_V2(uuid_struct,0x34,0x0a,0x1b,0x80, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define COPY_ENV_SENS_SERVICE_UUID(uuid_struct)  COPY_UUID_128_V2(uuid_struct,0x42,0x82,0x1a,0x40, 0xe4,0x77, 0x11,0xe2, 0x82,0xd0, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TEMP_CHAR_UUID(uuid_struct)         COPY_UUID_128_V2(uuid_struct,0xa3,0x2e,0x55,0x20, 0xe4,0x77, 0x11,0xe2, 0xa9,0xe3, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_PRESS_CHAR_UUID(uuid_struct)        COPY_UUID_128_V2(uuid_struct,0xcd,0x20,0xc4,0x80, 0xe4,0x8b, 0x11,0xe2, 0x84,0x0b, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_HUMIDITY_CHAR_UUID(uuid_struct)     COPY_UUID_128_V2(uuid_struct,0x01,0xc5,0x0b,0x60, 0xe4,0x8c, 0x11,0xe2, 0xa0,0x73, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)

// LED service
#define COPY_LED_SERVICE_UUID(uuid_struct)  COPY_UUID_128_V2(uuid_struct,0x0b,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_LED_UUID(uuid_struct)          COPY_UUID_128_V2(uuid_struct,0x0c,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	
// Input Service
#define COPY_INPUT_SERVICE_UUID(uuid_struct)  COPY_UUID_128_V2(uuid_struct,0x0d,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xd5,0x2b)
#define COPY_INPUT_CHAR_UUID(uuid_struct)          COPY_UUID_128_V2(uuid_struct,0x0e,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xd5,0x2b)

	
uint8_t BLE::SERVER_BDADDR[] = { 0x12, 0x34, 0x00, 0xE1, 0x80, 0x03 };
uint8_t BLE::bdaddr[BDADDR_SIZE] = { 0, 0, 0, 0, 0, 0};
uint8_t BLE::bnrg_expansion_board = IDB04A1;
AxesRaw_t BLE::axes_data[] = { 0, 0, 0 };
volatile uint8_t BLE::do_set_connectable = 1;
volatile uint16_t BLE::service_connection_handle = 0;
volatile uint8_t BLE::is_notification_enabled = FALSE;
volatile int BLE::connected = 0;
uint16_t BLE::TXCharHandle = 0;
uint16_t BLE::RXCharHandle = 0;
uint16_t BLE::accServHandle = 0;
uint16_t BLE::freeFallCharHandle = 0;
uint16_t BLE::accCharHandle = 0;
uint16_t BLE::envSensServHandle = 0;
uint16_t BLE::tempCharHandle = 0;
uint16_t BLE::pressCharHandle = 0;
uint16_t BLE::humidityCharHandle = 0;
uint16_t BLE::ledServHandle = 0;
uint16_t BLE::ledButtonCharHandle = 0;
uint16_t BLE::inputServHandle = 0;
uint16_t BLE::inputButtonCharHandle = 0;

void BLE::InitBLE()
{
	uint8_t  hwVersion = 0;
	uint16_t fwVersion = 0;
	uint16_t service_handle = 0;
	uint16_t dev_name_char_handle = 0;
	uint16_t appearance_char_handle = 0;
	const char *name = "BlueNRG";
	int ret = 0;
	
	/* Configure the User Button in GPIO Mode */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
	
	/* Initialize the BlueNRG SPI driver */
	BNRG_SPI_Init();
  
	/* Initialize the BlueNRG HCI */
	HCI_Init();

	  /* Reset BlueNRG hardware */
	BlueNRG_RST();
    
	/* get the BlueNRG HW and FW versions */
	getBlueNRGVersion(&hwVersion, &fwVersion);

	  /* 
	   * Reset BlueNRG again otherwise we won't
	   * be able to change its MAC address.
	   * aci_hal_write_config_data() must be the first
	   * command after reset otherwise it will fail.
	   */
	BlueNRG_RST();
  
	PRINTF("HWver %d, FWver %d", hwVersion, fwVersion);
  
	if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
		bnrg_expansion_board = IDB05A1; 
		/*
		 * Change the MAC address to avoid issues with Android cache:
		 * if different boards have the same MAC address, Android
		 * applications unless you restart Bluetooth on tablet/phone
		 */
		SERVER_BDADDR[5] = 0x02;
	}

	  /* The Nucleo board must be configured as SERVER */	
	//Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
	memcpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
  
	ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
		CONFIG_DATA_PUBADDR_LEN,
		bdaddr);
	if (ret) {
		PRINTF("Setting BD_ADDR failed.\n");
	}
  
	ret = aci_gatt_init();    
	if (ret) {
		PRINTF("GATT_Init failed.\n");
	}

	if (bnrg_expansion_board == IDB05A1) {
		ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}
	else {
		ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	}

	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("GAP_Init failed.\n");
	}

	ret = aci_gatt_update_char_value(service_handle,
		dev_name_char_handle,
		0,
		strlen(name),
		(uint8_t *)name);

	if (ret) {
		PRINTF("aci_gatt_update_char_value failed.\n");            
		while (1)
			;
	}
  
	ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
		OOB_AUTH_DATA_ABSENT,
		NULL,
		7,
		16,
		USE_FIXED_PIN_FOR_PAIRING,
		123456,
		BONDING);
	if (ret == BLE_STATUS_SUCCESS) {
		PRINTF("BLE Stack Initialized.\n");
	}
  
	PRINTF("SERVER: BLE Stack Initialized\n");
  
	ret = AddAccService();
  
	if (ret == BLE_STATUS_SUCCESS)
		PRINTF("Acc service added successfully.\n");
	else
		PRINTF("Error while adding Acc service.\n");
  
	ret = AddControlSensorService();
  
	if (ret == BLE_STATUS_SUCCESS)
		PRINTF("Environmental Sensor service added successfully.\n");
	else
		PRINTF("Error while adding Environmental Sensor service.\n");
	
	ret = AddLEDService();
	if (ret == BLE_STATUS_SUCCESS)
		PRINTF("LED service added successfully.\n");
	else
		PRINTF("Error while adding LED service.\n");
	
	ret = AddInputService();
	if (ret == BLE_STATUS_SUCCESS)
		PRINTF("Input service added successfully.\n");
	else
		PRINTF("Error while adding Input service.\n");

		  /* Set output power level */
	ret = aci_hal_set_tx_power_level(1, 4);
}

void BLE::Update()
{
	HCI_Process();
	User_Process();
}

tBleStatus BLE::AddAccService(void)
{
	tBleStatus ret;

	uint8_t uuid[16];
  
	COPY_ACC_SERVICE_UUID(uuid);
	ret = aci_gatt_add_serv(UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&accServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
	COPY_FREE_FALL_UUID(uuid);
	ret =  aci_gatt_add_char(accServHandle,
		UUID_TYPE_128,
		uuid,
		1,
		CHAR_PROP_NOTIFY,
		ATTR_PERMISSION_NONE,
		0,
		16,
		0,
		&freeFallCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
  
	COPY_ACC_UUID(uuid);  
	ret =  aci_gatt_add_char(accServHandle,
		UUID_TYPE_128,
		uuid,
		6,
		CHAR_PROP_NOTIFY|CHAR_PROP_READ,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
		16,
		0,
		&accCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
  
	PRINTF("Service ACC added. Handle 0x%04X, Free fall Charac handle: 0x%04X, Acc Charac handle: 0x%04X\n", accServHandle, freeFallCharHandle, accCharHandle);	
	return BLE_STATUS_SUCCESS; 
  
fail:
	PRINTF("Error while adding ACC service.\n");
	return BLE_STATUS_ERROR ;
    
}

/*
 * @brief  Add LED button service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus BLE::AddLEDService(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
  
	/* copy "LED service UUID" defined above to 'uuid' local variable */
	COPY_LED_SERVICE_UUID(uuid);
	
	ret = aci_gatt_add_serv(UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&ledServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
	/* copy "LED button characteristic UUID" defined above to 'uuid' local variable */  
	COPY_LED_UUID(uuid);
	
	ret =  aci_gatt_add_char(ledServHandle,
		UUID_TYPE_128,
		uuid,
		4,
		CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&ledButtonCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;  
  
	PRINTF("Service LED BUTTON added. Handle 0x%04X, LED button Charac handle: 0x%04X\n", ledServHandle, ledButtonCharHandle);	
	return BLE_STATUS_SUCCESS; 
  
fail:
	PRINTF("Error while adding LED service.\n");
	return BLE_STATUS_ERROR;
}

/*
 * @brief  Add Input button service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus BLE::AddInputService(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
  
	/* copy "Input service UUID" defined above to 'uuid' local variable */
	COPY_INPUT_SERVICE_UUID(uuid);
	
	ret = aci_gatt_add_serv(UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		7,
		&inputServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
	/* copy "INPUT button characteristic UUID" defined above to 'uuid' local variable */  
	COPY_INPUT_CHAR_UUID(uuid);
	
	ret =  aci_gatt_add_char(inputServHandle,
		UUID_TYPE_128,
		uuid,
		4,
		CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_ATTRIBUTE_WRITE,
		16,
		1,
		&inputButtonCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;  
  
	PRINTF("Service Input BUTTON added. Handle 0x%04X, Input button Charac handle: 0x%04X\n", inputServHandle, inputButtonCharHandle);	
	return BLE_STATUS_SUCCESS; 
  
fail:
	PRINTF("Error while adding INPUT service.\n");
	return BLE_STATUS_ERROR;
}

/**
 * @brief  Add the Control Sensor service.
 *
 * @param  None
 * @retval Status
 */
tBleStatus BLE::AddControlSensorService(void)
{
	tBleStatus ret;
	uint8_t uuid[16];
	uint16_t uuid16;
	charactFormat charFormat;
	uint16_t descHandle;
  
	COPY_ENV_SENS_SERVICE_UUID(uuid);
	ret = aci_gatt_add_serv(UUID_TYPE_128,
		uuid,
		PRIMARY_SERVICE,
		10,
		&envSensServHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
  
	/* Temperature Characteristic */
	COPY_TEMP_CHAR_UUID(uuid);  
	ret =  aci_gatt_add_char(envSensServHandle,
		UUID_TYPE_128,
		uuid,
		2,
		CHAR_PROP_READ,
		ATTR_PERMISSION_NONE,
		GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
		16,
		0,
		&tempCharHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
 
	charFormat.format = FORMAT_SINT16;
	charFormat.exp = -1;
	charFormat.unit = UNIT_TEMP_CELSIUS;
	charFormat.name_space = 0;
	charFormat.desc = 0;
  
	uuid16 = CHAR_FORMAT_DESC_UUID;
  
	ret = aci_gatt_add_char_desc(envSensServHandle,
		tempCharHandle,
		UUID_TYPE_16,
		(uint8_t *)&uuid16, 
		7,
		7,
		(void *)&charFormat, 
		ATTR_PERMISSION_NONE,
		ATTR_ACCESS_READ_ONLY,
		0,
		16,
		FALSE,
		&descHandle);
	if (ret != BLE_STATUS_SUCCESS) goto fail;
  
	/* Pressure Characteristic */
	if (1) { //FIXME
		COPY_PRESS_CHAR_UUID(uuid);  
		ret =  aci_gatt_add_char(envSensServHandle,
			UUID_TYPE_128,
			uuid,
			3,
			CHAR_PROP_READ,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
			16,
			0,
			&pressCharHandle);
		if (ret != BLE_STATUS_SUCCESS) goto fail;
    
		charFormat.format = FORMAT_SINT24;
		charFormat.exp = -5;
		charFormat.unit = UNIT_PRESSURE_BAR;
		charFormat.name_space = 0;
		charFormat.desc = 0;
    
		uuid16 = CHAR_FORMAT_DESC_UUID;
    
		ret = aci_gatt_add_char_desc(envSensServHandle,
			pressCharHandle,
			UUID_TYPE_16,
			(uint8_t *)&uuid16, 
			7,
			7,
			(void *)&charFormat, 
			ATTR_PERMISSION_NONE,
			ATTR_ACCESS_READ_ONLY,
			0,
			16,
			FALSE,
			&descHandle);
		if (ret != BLE_STATUS_SUCCESS) goto fail;
	}    
	/* Humidity Characteristic */
	if (1) {   //FIXME
		COPY_HUMIDITY_CHAR_UUID(uuid);  
		ret =  aci_gatt_add_char(envSensServHandle,
			UUID_TYPE_128,
			uuid,
			2,
			CHAR_PROP_READ,
			ATTR_PERMISSION_NONE,
			GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
			16,
			0,
			&humidityCharHandle);
		if (ret != BLE_STATUS_SUCCESS) goto fail;
    
		charFormat.format = FORMAT_UINT16;
		charFormat.exp = -1;
		charFormat.unit = UNIT_UNITLESS;
		charFormat.name_space = 0;
		charFormat.desc = 0;
    
		uuid16 = CHAR_FORMAT_DESC_UUID;
    
		ret = aci_gatt_add_char_desc(envSensServHandle,
			humidityCharHandle,
			UUID_TYPE_16,
			(uint8_t *)&uuid16, 
			7,
			7,
			(void *)&charFormat, 
			ATTR_PERMISSION_NONE,
			ATTR_ACCESS_READ_ONLY,
			0,
			16,
			FALSE,
			&descHandle);
		if (ret != BLE_STATUS_SUCCESS) goto fail;
	} 
	PRINTF("Service ENV_SENS added. Handle 0x%04X, TEMP Charac handle: 0x%04X, PRESS Charac handle: 0x%04X, HUMID Charac handle: 0x%04X\n", envSensServHandle, tempCharHandle, pressCharHandle, humidityCharHandle);	
	return BLE_STATUS_SUCCESS; 
  
fail:
	PRINTF("Error while adding ENV_SENS service.\n");
	return BLE_STATUS_ERROR ;
  
}

/**
 * @brief  Process user input (i.e. pressing the USER button on Nucleo board)
 *         and send the updated acceleration data to the remote client.
 *
 * @param  AxesRaw_t* p_axes
 * @retval None
 */
void BLE::User_Process()
{
	if (do_set_connectable) {
		setBLEConnectable();
		do_set_connectable = FALSE;
	}
}

/**
 * @brief  Puts the device in connectable mode.
 *         If you want to specify a UUID list in the advertising data, those data can
 *         be specified as a parameter in aci_gap_set_discoverable().
 *         For manufacture data, aci_gap_update_adv_data must be called.
 * @param  None 
 * @retval None
 */
/* Ex.:
 *
 *  tBleStatus ret;    
 *  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'B','l','u','e','N','R','G'};    
 *  const uint8_t serviceUUIDList[] = {AD_TYPE_16_BIT_SERV_UUID,0x34,0x12};    
 *  const uint8_t manuf_data[] = {4, AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0x05, 0x02, 0x01};
 *  
 *  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
 *                                 8, local_name, 3, serviceUUIDList, 0, 0);    
 *  ret = aci_gap_update_adv_data(5, manuf_data);
 *
 */
void BLE::setBLEConnectable(void)
{  
	tBleStatus ret;
  
	const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'l', 'u', 'e', 'N', 'R', 'G' };
  
	/* disable scan response */
	hci_le_set_scan_resp_data(0, NULL);
	PRINTF("General Discoverable Mode.\n");
  
	ret = aci_gap_set_discoverable(ADV_IND,
		0,
		0,
		PUBLIC_ADDR,
		NO_WHITE_LIST_USE,
		sizeof(local_name),
		local_name,
		0,
		NULL,
		0,
		0);
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error while setting discoverable mode (%d)\n", ret);    
	}  
}

/**
 * @brief  Send a notification for a Free Fall detection.
 *
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus BLE::Free_Fall_Notify(void)
{  
	uint8_t val;
	tBleStatus ret;

	val = 0x01;	
	ret = aci_gatt_update_char_value(accServHandle,
		freeFallCharHandle,
		0,
		1,
		&val);
	
	if (ret != BLE_STATUS_SUCCESS) 
	{
		PRINTF("Error while updating FFall characteristic.\n");
		return BLE_STATUS_ERROR ;
	}
	return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Update acceleration characteristic value.
 *
 * @param  Structure containing acceleration value in mg
 * @retval Status
 */
tBleStatus BLE::AccUpdate(AxesRaw_t *data)
{  
	tBleStatus ret;    
	uint8_t buff[6];
    
	STORE_LE_16(buff, data->AXIS_X);
	STORE_LE_16(buff + 2, data->AXIS_Y);
	STORE_LE_16(buff + 4, data->AXIS_Z);
	
	ret = aci_gatt_update_char_value(accServHandle, accCharHandle, 0, 6, buff);
	
	if (ret != BLE_STATUS_SUCCESS) 
	{
		PRINTF("Error while updating ACC characteristic.\n");
		return BLE_STATUS_ERROR ;
	}
	return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void BLE::GAPConnectionCompleteCB(uint8_t addr[6], uint16_t handle)
{  
	connected = TRUE;
	service_connection_handle = handle;
  
	PRINTF("Connected to device:");
	for (int i = 5; i > 0; i--) 
	{
		PRINTF("%02X-", addr[i]);
	}
	PRINTF("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None 
 * @retval None
 */
void BLE::GAPDisconnectionCompleteCB(void)
{
	connected = FALSE;
	PRINTF("Disconnected\n");
	/* Make the device connectable again. */
	do_set_connectable = TRUE;
	is_notification_enabled = FALSE;
}

/**
 * @brief  Read request callback.
 * @param  uint16_t Handle of the attribute
 * @retval None
 */
void BLE::Read_Request_CB(uint16_t handle)
{  
	if (handle == accCharHandle + 1) {
		AccUpdate((AxesRaw_t*)&axes_data);
	}  
	else if (handle == tempCharHandle + 1) {
		int16_t data;
		data = 270 + ((uint64_t)rand() * 15) / RAND_MAX; //sensor emulation        
		AccUpdate((AxesRaw_t*)&axes_data); //FIXME: to overcome issue on Android App
		                                    // If the user button is not pressed within
		                                    // a short time after the connection,
		                                    // a pop-up reports a "No valid characteristics found" error.
	}
  
	//EXIT:
	if (service_connection_handle != 0)
		aci_gatt_allow_read(service_connection_handle);
}

/**
 * @brief  This function is called when an attribute characteristic is modified.
 * @param  Handle of the attribute
 * @param  Size of the modified attribute data
 * @param  Pointer to the modified attribute data
 * @retval None
 */
void BLE::Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  /* If GATT client has modified 'LED button characteristic' value, toggle LED2 */
	if (handle == inputButtonCharHandle + 1)
	{
		/*
		Format:
			1st byte - Motor Index Mask (0x0 - 0xF)
			2nd byte - value
		*/
		
		if (data_length < 3)
			return;
		
		uint8_t motorIndex = att_data[0];
		uint8_t value = att_data[1];
		uint8_t direction = att_data[2];
		
		//PRINTF("indx: %u, value: %u, dir: %u\n", motorIndex, value, direction);
		ControlManager::Instance()->UpdateMotor(motorIndex, value, direction);
	}
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
	hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
	/* obtain event packet */
	hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
	if (hci_pckt->type != HCI_EVENT_PKT)
		return;
  
	switch (event_pckt->evt) {
    
	case EVT_DISCONN_COMPLETE:
		{
			BLE::GAPDisconnectionCompleteCB();
		}
		break;
    
	case EVT_LE_META_EVENT:
		{
			evt_le_meta_event *evt = (evt_le_meta_event*)event_pckt->data;
      
			switch (evt->subevent) {
			case EVT_LE_CONN_COMPLETE:
				{
					evt_le_connection_complete *cc = (evt_le_connection_complete*)evt->data;
					BLE::GAPConnectionCompleteCB(cc->peer_bdaddr, cc->handle);
				}
				break;
			}
		}
		break;
    
	case EVT_VENDOR:
		{
			evt_blue_aci *blue_evt = (evt_blue_aci*)event_pckt->data;
			switch (blue_evt->ecode) {

			// GATT Attribute modification
			case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
				{
					if (BLE::GetExpansionBoard() == IDB05A1) {
						evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
						BLE::Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); 
					}
					else {
						evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
						BLE::Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); 
					}                       
				}
				break;

			case EVT_BLUE_GATT_READ_PERMIT_REQ:
				{
					evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req*)blue_evt->data;                    
					BLE::Read_Request_CB(pr->attr_handle);                    
				}
				break;
			}
		}
		break;
	}    
}