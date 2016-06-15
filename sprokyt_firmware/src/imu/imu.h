// imu.h


#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>

void InitIMU(void);
void UpdateIMU(void);

void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss);


#endif // _IMU_H_