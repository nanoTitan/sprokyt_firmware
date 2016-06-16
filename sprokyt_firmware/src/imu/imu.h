// imu.h

#ifndef _IMU_H_
#define _IMU_H_

#ifdef __cplusplus
 extern "C" {
#endif 

#include <stdint.h>

void InitIMU(void);
void UpdateIMU(void);

#ifdef __cplusplus
}
#endif

#endif // _IMU_H_