/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_IMU_H_
#define INCLUDE_FLIGHT_IMU_H_

#include "flight/global_defs.h"

namespace imu {

void Init();
void Calibrate();
void AttachCallback(void (*function)());
void Read(ImuData *imu);

}  // namespace imu

#endif  // INCLUDE_FLIGHT_IMU_H_
