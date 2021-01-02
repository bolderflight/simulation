/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_VMS_H_
#define INCLUDE_FLIGHT_VMS_H_

#include "flight/global_defs.h"

namespace vms {

void Init();
void Run(const InceptorData &inceptor, const SensorData &sensor, SenProcData *sen_proc, AuxData *aux, EffectorCmds *effector);

}  // namespace vms

#endif  // INCLUDE_FLIGHT_VMS_H_
