
/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include "main.h"
#include "cube_hal.h"
#include "ControlManager.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include "debug.h"

extern "C" {
#include "stm32f4xx_nucleo.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_accelero.h"
#include "x_nucleo_iks01a1_gyro.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_humidity.h"
#include "x_nucleo_iks01a1_temperature.h"
#include "error.h"
}
/* Private define ------------------------------------------------------------*/

/* Definition for TIMx's NVIC */
#define SF_TIM_COUNTER_CLK					 10000
#define SF_TIM_PERIOD						 10

// Enable sensor masks
#define PRESSURE_SENSOR                         0x00000001
#define TEMPERATURE_SENSOR                      0x00000002
#define HUMIDITY_SENSOR                         0x00000004
#define UV_SENSOR                               0x00000008  // for future use
#define ACCELEROMETER_SENSOR                    0x00000010
#define GYROSCOPE_SENSOR                        0x00000020
#define MAGNETIC_SENSOR                         0x00000040

#define OSXMOTIONFX_CHECK_CALIBRATION ((uint32_t)0x12345678)

#ifdef OSXMOTIONFX_STORE_CALIB_FLASH

#if defined(USE_STM32F4XX_NUCLEO)
#define OSXMOTIONFX_FLASH_ADD ((uint32_t)0x08060000)
#define OSXMOTIONFX_FLASH_SECTOR FLASH_SECTOR_7
#endif

#if defined(USE_STM32L4XX_NUCLEO)
#define OSXMOTIONFX_FLASH_ADD ((uint32_t)0x080FF800)
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);
#endif

#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
#if defined (__IAR_SYSTEMS_ICC__)
__no_init uint32_t CalibrationStructureRAM[8];
#elif defined (__CC_ARM)
#if (defined (USE_STM32F4XX_NUCLEO))
uint32_t *CalibrationStructureRAM = (uint32_t *)0x20017FC0;
#endif

#if (defined (USE_STM32L4XX_NUCLEO))
uint32_t *CalibrationStructureRAM = (uint32_t *)0x10000000;
#endif
#elif defined (__GNUC__)
uint32_t CalibrationStructureRAM[8] __attribute__((section(".noinit")));
#else
#error "Toolchain not supported"
#endif
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive;

/* Private variables ---------------------------------------------------------*/
osxMFX_calibFactor magOffset;
SensorAxes_t ACC_Value;
SensorAxes_t GYR_Value;
SensorAxes_t MAG_Value;
TIM_HandleTypeDef    SFTimHandle;
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
void *MAGNETO_handle = NULL;
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use X_NUCLEO_IKS01A1 expansion board to send sensor fusion data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on Sensors_DataLog specific application,
 *         which is developed by STMicroelectronics and provided with X-CUBE-MEMS1 package.
 *
 *         After connection has been established with Sensors_DataLog application provided with X-CUBE-MEMS1 package:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity and Pressure
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyroscope and Magnetometer
 *         - the user can also visualize this data as graphs
 *         - the user can also visualize a cube animation that shows sensor fusion outputs
 *         - the user can also visualize MEMS data (Accelerometer, Gyroscope and Magnetometer) and sensor fusion data (quaternions and Euler angles) when sensor fusion is activated
 *         - the user can calibrate the Magnetometer when sensor fusion is activated pressing on the user button; the calibration data are stored in memory; this procedure
 *           is needed only when fusion 9X is used
 * @param  None
 * @retval Integer
 */
IMU::IMU()
:PRESSURE_Value(0),
 HUMIDITY_Value(0),
 TEMPERATURE_Value(0),
 SF_Active(0),
 SF_6x_enabled(0),
 SF_change(0),
 sysclk(0),
 hclk(0),
 pclk1(0),
 pclk2(0),
 calibIndex(0),         // run calibration @ 25Hz
 isCal(0)
{
	_eulerAngles.x = _eulerAngles.y = _eulerAngles.z = 0;
	_quaternion.w = _quaternion.x = _quaternion.y = _quaternion.z = 0;
}

IMU::~IMU()
{	
}

IMU* IMU::Instance()
{
	static IMU* pInstance = NULL;
	if (!pInstance)
	{
		pInstance = new IMU();
	}
	
	return pInstance;
}

void IMU::InitIMU()
{	
	InitializeAllSensors();
	EnableAllSensors();
  
	MotionFX_manager_init();
  
	MotionFX_manager_start_9X();
  
	sysclk = HAL_RCC_GetSysClockFreq();
	hclk = HAL_RCC_GetHCLKFreq();
	pclk1 = HAL_RCC_GetPCLK1Freq();
	pclk2 = HAL_RCC_GetPCLK2Freq();
  
	/* Check if the calibration is already available in memory */
	RecallCalibrationFromMemory();
	
	/*Initialize the sensor fusion*/ 
	SFTimerInit();
}

void IMU::UpdateIMU(void)
{
	/* Check if user button was pressed only when Sensor Fusion is active */
	if ((BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET) /*&& SF_Active*/)
	{
		while (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
			;
		
		/* Reset the Compass Calibration */
		isCal = 0;
		osx_MotionFX_compass_forceReCalibration();
      
		ResetCalibrationInMemory();
      
		magOffset.magOffX = 0;
		magOffset.magOffY = 0;
		magOffset.magOffZ = 0;
      
		/* Switch off the LED */
		BSP_LED_Off(LED2);
	}
    
	if (!SF_Active)
	{
		Pressure_Sensor_Handler();
		Accelero_Sensor_Handler();
		Gyro_Sensor_Handler();
		Magneto_Sensor_Handler();
	}
	
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
void IMU::InitializeAllSensors(void)
{
  /* Try to use LSM6DS3 DIL24 if present, otherwise use LSM6DS0 on board */
	DrvStatusTypeDef result = BSP_ACCELERO_Init(ACCELERO_SENSORS_AUTO, &ACCELERO_handle);		// ACCELERO_SENSORS_AUTO, LSM6DS3_X_0, LSM6DS0_X_0
	if (result != COMPONENT_OK)
		Error_Handler();
	
	/* Try to use LSM6DS3 if present, otherwise use LSM6DS0 */
	result = BSP_GYRO_Init(GYRO_SENSORS_AUTO, &GYRO_handle);
	if (result != COMPONENT_OK)
		Error_Handler();
	
	/* Force to use LIS3MDL */
	result = BSP_MAGNETO_Init(LIS3MDL_0, &MAGNETO_handle);
	if (result != COMPONENT_OK)
		Error_Handler();
	
	/* Force to use HTS221 */
	result = BSP_HUMIDITY_Init(HTS221_H_0, &HUMIDITY_handle);
	if (result != COMPONENT_OK)
		Error_Handler();
	
	/* Force to use HTS221 */
	result = BSP_TEMPERATURE_Init(HTS221_T_0, &TEMPERATURE_handle);
	if (result != COMPONENT_OK)
		Error_Handler();
	
	/* Try to use LPS22HB DIL24 or LPS25HB DIL24 if present, otherwise use LPS25HB on board */
	result = BSP_PRESSURE_Init(PRESSURE_SENSORS_AUTO, &PRESSURE_handle);
	if (result != COMPONENT_OK)
		Error_Handler();
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
void IMU::EnableAllSensors(void)
{
	BSP_ACCELERO_Sensor_Enable(ACCELERO_handle);
	BSP_GYRO_Sensor_Enable(GYRO_handle);
	BSP_MAGNETO_Sensor_Enable(MAGNETO_handle);
	BSP_HUMIDITY_Sensor_Enable(HUMIDITY_handle);
	BSP_TEMPERATURE_Sensor_Enable(TEMPERATURE_handle);
	BSP_PRESSURE_Sensor_Enable(PRESSURE_handle);
}


/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
void IMU::FloatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
	*out_int = (int32_t)in;
	if (in >= 0.0f)
	{
		in = in - (float)(*out_int);
	}
	else
	{
		in = (float)(*out_int) - in;
	}
	*out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg ACCELERO part of the stream
 * @retval None
 */
void IMU::Accelero_Sensor_Handler()
{
	uint8_t status = 0;
  
	if (ACCELEROMETER_SENSOR && BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
		//PRINTF("Accel x: %d, y: %d, z: %d\n", ACC_Value.AXIS_X, ACC_Value.AXIS_Y, ACC_Value.AXIS_Z);
	}
}

/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg GYRO part of the stream
 * @retval None
 */
void IMU::Gyro_Sensor_Handler()
{
	uint8_t status = 0;
  
	if (GYROSCOPE_SENSOR && BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
		//PRINTF("Gyro x: %d, y: %d, z: %d\n", GYR_Value.AXIS_X, GYR_Value.AXIS_Y, GYR_Value.AXIS_Z);
	}
}

/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg MAGNETO part of the stream
 * @retval None
 */
void IMU::Magneto_Sensor_Handler()
{
	uint8_t status = 0;
  
	if (BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
		//PRINTF("Mag x: %d, y: %d, z: %d\n", MAG_Value.AXIS_X, MAG_Value.AXIS_Y, MAG_Value.AXIS_Z);
	}
}

/**
 * @brief  Handles the PRESSURE sensor data getting/sending
 * @param  Msg PRESSURE part of the stream
 * @retval None
 */
void IMU::Pressure_Sensor_Handler()
{
	int32_t d1, d2;
	uint8_t status = 0;
  
	if (PRESSURE_SENSOR && BSP_PRESSURE_IsInitialized(PRESSURE_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_PRESSURE_Get_Press(PRESSURE_handle, &PRESSURE_Value);
		//PRINTF("Pressure: %f\n", PRESSURE_Value);
	}
}

/**
 * @brief  Handles the HUMIDITY sensor data getting/sending
 * @param  Msg HUMIDITY part of the stream
 * @retval None
 */
void IMU::Humidity_Sensor_Handler()
{
	int32_t d1, d2;
	uint8_t status = 0;
  
	if (HUMIDITY_SENSOR && BSP_HUMIDITY_IsInitialized(HUMIDITY_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_HUMIDITY_Get_Hum(HUMIDITY_handle, &HUMIDITY_Value);
		PRINTF("Humidity: %f\n", HUMIDITY_Value);
	}
}

/**
 * @brief  Handles the TEMPERATURE sensor data getting/sending
 * @param  Msg TEMPERATURE part of the stream
 * @retval None
 */
void IMU::Temperature_Sensor_Handler()
{
	int32_t d3, d4;
	uint8_t status = 0;
  
	if (TEMPERATURE_SENSOR && BSP_TEMPERATURE_IsInitialized(TEMPERATURE_handle, &status) == COMPONENT_OK && status == 1)
	{
		BSP_TEMPERATURE_Get_Temp(TEMPERATURE_handle, &TEMPERATURE_Value);
		PRINTF("Temperature: %f\n", TEMPERATURE_Value);
	}
}

/**
 * @brief  Initialize Sensor Fusion timer
 * @param  None
 * @retval None
 */
void IMU::SFTimerInit()
{
	TIM_OC_InitTypeDef sConfig;
	uint16_t uhPrescalerValue;
	//uint16_t uhCapturedValue = 0;
  
	/* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
	uhPrescalerValue = (uint16_t)((SystemCoreClock) / SF_TIM_COUNTER_CLK) - 1;
  
	/*##-1- Configure the TIM peripheral #######################################*/
	/* Set TIMx instance */
	SFTimHandle.Instance = TIM_SF;
  
	/* Initialize TIM3 peripheral as follow:
	+ Period = 65535
	+ Prescaler = (SystemCoreClock/2)/60000
	+ ClockDivision = 0
	+ Counter direction = Up
	*/
  
	uint32_t timer_period = (SF_TIM_COUNTER_CLK / (1000 / SF_TIM_PERIOD)) - 1;
	SFTimHandle.Init.Period = timer_period;
	SFTimHandle.Init.Prescaler = uhPrescalerValue;
	SFTimHandle.Init.ClockDivision = 0;
	SFTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	SFTimHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_OC_Init(&SFTimHandle) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}
  
	/*##-2- Configure the PWM channels #########################################*/
	/* Common configuration */
	sConfig.OCMode = TIM_OCMODE_TIMING;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_SET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;
  
	/* Set the pulse value for channel 1 */
	sConfig.Pulse = timer_period / 2;
  
	if (HAL_TIM_OC_ConfigChannel(&SFTimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}
  
  
	/*##-4- Start the Output Compare mode in interrupt mode ####################*/
	/* Start Channel1 */
	if (HAL_TIM_OC_Start_IT(&SFTimHandle, TIM_CHANNEL_1) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}
	
	SF_Active = 1;
}

/**
 * @brief  Handles the Sensor Fusion
 * @param  Msg Sensor Fusion part of the stream
 * @retval None
 */
void IMU::SF_Handler()
{
	uint8_t status_acc = 0;
	uint8_t status_gyr = 0;
	uint8_t status_mag = 0;
	
	//PRINTF("SF_Active: %d\n", (int)SF_Active);
  
	if (SF_Active)
	{ 
		BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status_acc);
		BSP_GYRO_IsInitialized(GYRO_handle, &status_gyr);
		BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status_mag);
  
		if (status_acc && status_gyr && status_mag)
		{
			if (SF_change == 1)
			{
				if (SF_6x_enabled == 0)
				{
					MotionFX_manager_stop_9X();
					MotionFX_manager_start_6X();
					SF_6x_enabled = 1;
				}
				else
				{
					MotionFX_manager_stop_6X();
					MotionFX_manager_start_9X();
					SF_6x_enabled = 0;
				}
				SF_change = 0;
			}
      
			BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
			BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
			BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
      
			MotionFX_manager_run();
      
			/* Check if is calibrated */
			if (isCal != 0x01)
			{
			  /* Run Compass Calibration @ 25Hz */
				calibIndex++;
				if (calibIndex == 4)
				{
					SensorAxes_t ACC_Loc, MAG_Loc;
					calibIndex = 0;
					ACC_Loc.AXIS_X = ACC_Value.AXIS_X;
					ACC_Loc.AXIS_Y = ACC_Value.AXIS_Y;
					ACC_Loc.AXIS_Z = ACC_Value.AXIS_Z;
					MAG_Loc.AXIS_X = MAG_Value.AXIS_X;
					MAG_Loc.AXIS_Y = MAG_Value.AXIS_Y;
					MAG_Loc.AXIS_Z = MAG_Value.AXIS_Z;
					osx_MotionFX_compass_saveAcc(ACC_Loc.AXIS_X, ACC_Loc.AXIS_Y, ACC_Loc.AXIS_Z); /* Accelerometer data ENU systems coordinate  */
					osx_MotionFX_compass_saveMag(MAG_Loc.AXIS_X, MAG_Loc.AXIS_Y, MAG_Loc.AXIS_Z); /* Magnetometer data ENU systems coordinate */
					osx_MotionFX_compass_run();
				}
        
				/* Check if is calibrated */
				isCal = osx_MotionFX_compass_isCalibrated();
				if (isCal == 0x01)
				{
				  /* Get new magnetometer offset */
					osx_MotionFX_getCalibrationData(&magOffset);
          
					/* Save the calibration in Memory */
					SaveCalibrationToMemory();
          
					/* Switch on the Led */
					BSP_LED_On(LED2);
				}
			}
      
			osxMFX_output *MotionFX_Engine_Out = MotionFX_manager_getDataOUT();
      
			if (SF_6x_enabled == 1)
			{
				//memcpy(&_eulerAngles, (uint8_t*)&MotionFX_Engine_Out->rotation_6X, 3 * sizeof(float));  // Euler rotation
				//memcpy(&_quaternion, (uint8_t*)&MotionFX_Engine_Out->quaternion_6X, 4 * sizeof(float)); // Quaternion
				
				ControlManager::Instance()->GetController()->UpdateIMU(
					MotionFX_Engine_Out->rotation_6X[0],
					MotionFX_Engine_Out->rotation_6X[1],
					MotionFX_Engine_Out->rotation_6X[2]);
			}
			else
			{
				//memcpy(&_eulerAngles, (uint8_t*)&MotionFX_Engine_Out->rotation_9X, 3 * sizeof(float));  // Euler rotation
				//memcpy(&_quaternion, (uint8_t*)&MotionFX_Engine_Out->quaternion_9X, 4 * sizeof(float)); // Quaternion
				
				ControlManager::Instance()->GetController()->UpdateIMU(
					MotionFX_Engine_Out->rotation_9X[0],
					MotionFX_Engine_Out->rotation_9X[1],
					MotionFX_Engine_Out->rotation_9X[2]);
			}
		}
	}
}

/**
* @brief  Conversion complete callback in non blocking mode
* @param  htim timer handler
* @retval None
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	IMU::Instance()->SF_Handler();
}

/**
 * @brief  Get the current tick value in millisecond
 * @param  None
 * @retval The tick value
 */
uint32_t user_currentTimeGetTick(void)
{
	return HAL_GetTick();
}

/**
 * @brief  Get the delta tick value in millisecond from Tick1 to the current tick
 * @param  Tick1 the reference tick used to compute the delta
 * @retval The delta tick value
 */
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1)
{
	volatile uint32_t Delta, Tick2;
  
	Tick2 = HAL_GetTick();
  
	/* Capture computation */
	Delta = Tick2 - Tick1;
	return Delta;
}

/**
 * @brief  Save the Magnetometer calibration values to memory
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
unsigned char IMU::SaveCalibrationToMemory(void)
{
	unsigned char Success = 1;
  
	/* Reset Before The data in Memory */
	Success = ResetCalibrationInMemory();
  
	if (Success)
#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
	{
	  /* Store in Flash Memory */
		uint32_t Address = OSXMOTIONFX_FLASH_ADD;
#if (defined (USE_STM32F4XX_NUCLEO))
		uint32_t WritemagOffset[8];
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
		uint64_t WritemagOffset64[8];
#endif
		int32_t WriteIndex;
#if (defined (USE_STM32F4XX_NUCLEO))
		WritemagOffset[0] = OSXMOTIONFX_CHECK_CALIBRATION;
		WritemagOffset[1] = (uint32_t) magOffset.magOffX;
		WritemagOffset[2] = (uint32_t) magOffset.magOffY;
		WritemagOffset[3] = (uint32_t) magOffset.magOffZ;
		memcpy(&WritemagOffset[4], &(magOffset.magGainX), sizeof(float));
		memcpy(&WritemagOffset[5], &(magOffset.magGainY), sizeof(float));
		memcpy(&WritemagOffset[6], &(magOffset.magGainZ), sizeof(float));
		memcpy(&WritemagOffset[7], &(magOffset.expMagVect), sizeof(float));
#endif
    
#if (defined (USE_STM32L4XX_NUCLEO))
		WritemagOffset64[0] = (uint64_t)OSXMOTIONFX_CHECK_CALIBRATION;
		WritemagOffset64[1] = (uint64_t) magOffset.magOffX;
		WritemagOffset64[2] = (uint64_t) magOffset.magOffY;
		WritemagOffset64[3] = (uint64_t) magOffset.magOffZ;
		memcpy(&WritemagOffset64[4], &(magOffset.magGainX), sizeof(float));
		memcpy(&WritemagOffset64[5], &(magOffset.magGainY), sizeof(float));
		memcpy(&WritemagOffset64[6], &(magOffset.magGainZ), sizeof(float));
		memcpy(&WritemagOffset64[7], &(magOffset.expMagVect), sizeof(float));
#endif
    
		    /* Unlock the Flash to enable the flash control register access *************/
		HAL_FLASH_Unlock();
    
#if (defined (USE_STM32F4XX_NUCLEO))
		for (WriteIndex = 0; WriteIndex < 8; WriteIndex++)
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
			for (WriteIndex = 0; WriteIndex < 8; WriteIndex++)
#endif
			{
#if (defined (USE_STM32F4XX_NUCLEO))
				if (HAL_FLASH_Program(TYPEPROGRAM_WORD, Address, WritemagOffset[WriteIndex]) == HAL_OK)
				{
					Address = Address + 4;
				}
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, WritemagOffset64[WriteIndex]) == HAL_OK)
				{
					Address = Address + 8;
				}
#endif
				else
				{
				  /* Error occurred while writing data in Flash memory.
				     User can add here some code to deal with this error */
				  /*
				     FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
				  */
					Error_Handler();
				}
			}
      
			/* Lock the Flash to disable the flash control register access (recommended
			 to protect the FLASH memory against possible unwanted operation) *********/
		HAL_FLASH_Lock();
	}
#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
	{
	  /* Store in RAM */
		CalibrationStructureRAM[0] = OSXMOTIONFX_CHECK_CALIBRATION;
		CalibrationStructureRAM[1] = (uint32_t) magOffset.magOffX;
		CalibrationStructureRAM[2] = (uint32_t) magOffset.magOffY;
		CalibrationStructureRAM[3] = (uint32_t) magOffset.magOffZ;
		memcpy(&CalibrationStructureRAM[4], &(magOffset.magGainX), sizeof(float));
		memcpy(&CalibrationStructureRAM[5], &(magOffset.magGainY), sizeof(float));
		memcpy(&CalibrationStructureRAM[6], &(magOffset.magGainZ), sizeof(float));
		memcpy(&CalibrationStructureRAM[7], &(magOffset.expMagVect), sizeof(float));
	}
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */
  
  
	return Success;
}

/**
 * @brief  Reset the Magnetometer calibration values in memory
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
unsigned char IMU::ResetCalibrationInMemory(void)
#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
{
  /* Reset Calibration Values in FLASH */
	unsigned char Success = 1;
  
	/* Erase First Flash sector */
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;
  
#if (defined (USE_STM32F4XX_NUCLEO))
	EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = OSXMOTIONFX_FLASH_SECTOR;
	EraseInitStruct.NbSectors = 1;
#endif
  
#if (defined (USE_STM32L4XX_NUCLEO))
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Banks       = GetBank(OSXMOTIONFX_FLASH_ADD);
	EraseInitStruct.Page        = GetPage(OSXMOTIONFX_FLASH_ADD);
	EraseInitStruct.NbPages     = 1;
#endif
  
	  /* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();
  
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
	  /*
	    Error occurred while sector erase.
	    User can add here some code to deal with this error.
	    SectorError will contain the faulty sector and then to know the code error on this sector,
	    user can call function 'HAL_FLASH_GetError()'
	*/
	/*
	  FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
	*/
		Error_Handler();
		Success = 0;
	}
  
	/* Lock the Flash to disable the flash control register access (recommended
	to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();
  
	return Success;
}
#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
{
  /* Reset Calibration Values in RAM */
	unsigned char Success = 1;
	int32_t Counter;
  
	for (Counter = 0; Counter < 8; Counter++)
		CalibrationStructureRAM[Counter] = 0xFFFFFFFF;
    
	return Success;
}
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

/**
 * @brief  Check if there are valid calibration values in memory and read them
 * @param  None
 * @retval 1 in case of success, 0 otherwise
 */
unsigned char IMU::RecallCalibrationFromMemory(void)
#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
{
  /* ReLoad the Calibration Values from FLASH */
	unsigned char Success = 1;
	uint32_t Address = OSXMOTIONFX_FLASH_ADD;
#if (defined (USE_STM32F4XX_NUCLEO))
	__IO uint32_t data = *(__IO uint32_t*) Address;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
	__IO uint64_t data = *(__IO uint64_t*) Address;
#endif
	if (data == OSXMOTIONFX_CHECK_CALIBRATION)
	{
		int32_t ReadIndex;
#if (defined (USE_STM32F4XX_NUCLEO))
		uint32_t ReadmagOffset[7];
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
		uint64_t ReadmagOffset[7];
#endif
    
		for (ReadIndex = 0; ReadIndex < 7; ReadIndex++)
		{
#if (defined (USE_STM32F4XX_NUCLEO))
			Address += 4;
			data = *(__IO uint32_t*) Address;
#endif
#if (defined (USE_STM32L4XX_NUCLEO))
			Address += 8;
			data = *(__IO uint64_t*) Address;
#endif
			ReadmagOffset[ReadIndex] = data;
		}
    
		magOffset.magOffX    = (signed short) ReadmagOffset[0];
		magOffset.magOffY    = (signed short) ReadmagOffset[1];
		magOffset.magOffZ    = (signed short) ReadmagOffset[2];
		magOffset.magGainX = (float) ReadmagOffset[3];
		magOffset.magGainY = (float) ReadmagOffset[4];
		magOffset.magGainZ = (float) ReadmagOffset[5];
		magOffset.expMagVect = (float) ReadmagOffset[6];
    
		/* Set the Calibration Structure */
		osx_MotionFX_setCalibrationData(&magOffset);
    
		/* Control the calibration status */
		isCal = osx_MotionFX_compass_isCalibrated();
    
		if (isCal == 0x01)
		{
		  /* Switch on the Led */
			BSP_LED_On(LED2);
		}
		else
		{
		  /* Switch off the Led */
			BSP_LED_Off(LED2);
		}
	}
	else
	{
	  /* Switch off the Led */
		BSP_LED_Off(LED2);
		isCal = 0;
	}
  
	return Success;
}
#else /* OSXMOTIONFX_STORE_CALIB_FLASH */
{
  /* ReLoad the Calibration Values from RAM */
	unsigned char Success = 1;
  
	if (CalibrationStructureRAM[0] == OSXMOTIONFX_CHECK_CALIBRATION)
	{
		magOffset.magOffX    = (signed short) CalibrationStructureRAM[1];
		magOffset.magOffY    = (signed short) CalibrationStructureRAM[2];
		magOffset.magOffZ    = (signed short) CalibrationStructureRAM[3];
		memcpy(&magOffset.magGainX, &(CalibrationStructureRAM[4]), sizeof(uint32_t));
		memcpy(&magOffset.magGainY, &(CalibrationStructureRAM[5]), sizeof(uint32_t));
		memcpy(&magOffset.magGainZ, &(CalibrationStructureRAM[6]), sizeof(uint32_t));
		memcpy(&magOffset.expMagVect, &(CalibrationStructureRAM[7]), sizeof(uint32_t));
    
		/* Set the Calibration Structure */
		osx_MotionFX_setCalibrationData(&magOffset);
    
		/* Control the calibration status */
		isCal = osx_MotionFX_compass_isCalibrated();
    
		if (isCal == 0x01)
		{
		  /* Switch on the Led */
			BSP_LED_On(LED2);
		}
		else
		{
		  /* Switch off the Led */
			BSP_LED_Off(LED2);
		}
	}
	else
	{
	  /* Switch off the Led */
		BSP_LED_Off(LED2);
		isCal = 0;
	}
  
	return Success;
}
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

#ifdef OSXMOTIONFX_STORE_CALIB_FLASH
#if (defined (USE_STM32L4XX_NUCLEO))
/**
  * @brief  Gets the page of a given address
  * @param  Addr Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
	uint32_t page = 0;
  
	if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
	{
	  /* Bank 1 */
		page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
	}
	else
	{
	  /* Bank 2 */
		page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
	}
  
	return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
	uint32_t bank = 0;
  
	if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
	{
	  /* No Bank swap */
		if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
		{
			bank = FLASH_BANK_1;
		}
		else
		{
			bank = FLASH_BANK_2;
		}
	}
	else
	{
	  /* Bank swap */
		if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
		{
			bank = FLASH_BANK_2;
		}
		else
		{
			bank = FLASH_BANK_1;
		}
	}
  
	return bank;
}
#endif /* USE_STM32L4XX_NUCLEO */
#endif /* OSXMOTIONFX_STORE_CALIB_FLASH */

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
