// imu.h

#ifndef _IMU_H_
#define _IMU_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>

void Init_IMU(void);
void UpdateIMU(void);
void Start_IMU();

#ifdef __cplusplus
}
#endif

#endif // _IMU_H_