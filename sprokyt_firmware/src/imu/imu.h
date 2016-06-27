// imu.h

#ifndef _IMU_H_
#define _IMU_H_

#include "mbed.h"
#include "math_ext.h"
#include <stdint.h>

extern "C" {
#include "sensor.h"
#include "MotionFX_Manager.h"
}

class IMU
{
public:
	static IMU* Instance();
	~IMU();
	
	void InitIMU();
	void UpdateIMU(void);
	void SFTimerInit();
	void InitSFTicker();
	
	static void SF_Handler();
	
private:
	IMU();
	
	void CalibrateSensorFusion();
	void UpdateSensorFusion();
	void InitializeAllSensors();
	void EnableAllSensors();
	void FloatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
	void Accelero_Sensor_Handler();
	void Gyro_Sensor_Handler();
	void Magneto_Sensor_Handler();
	void Pressure_Sensor_Handler();
	void Humidity_Sensor_Handler();
	void Temperature_Sensor_Handler();
	
	unsigned char SaveCalibrationToMemory(void);
	unsigned char ResetCalibrationInMemory();
	unsigned char RecallCalibrationFromMemory();
		
	Ticker m_sfTicker;
	float PRESSURE_Value;
	float HUMIDITY_Value;
	float TEMPERATURE_Value;
	volatile uint8_t SF_Active;
	volatile uint8_t SF_6x_enabled;
	volatile uint8_t SF_change;
	EulerAngle_t _eulerAngles;
	Quaternion_t _quaternion;
	uint32_t sysclk;
	uint32_t hclk;
	uint32_t pclk1;
	uint32_t pclk2;	
	uint8_t calibIndex;         // run calibration @ 25Hz
	uint8_t isCal;
};

#endif // _IMU_H_