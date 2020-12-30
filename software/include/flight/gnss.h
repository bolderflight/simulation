/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_GNSS_H_
#define INCLUDE_FLIGHT_GNSS_H_

#include "flight/global_defs.h"

namespace gnss {

void Init();
void Read(GnssData *ptr);

}  // namespace gnss

#endif  // INCLUDE_FLIGHT_GNSS_H_
