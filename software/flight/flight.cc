/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "core/core.h"
#include "flight/print_msg.h"
#include "flight/airdata.h"
#include "flight/imu.h"
#include "flight/gnss.h"


/* IMU data ready interrupt */
void drdy() {

}

int main() {
  /* Init debug messages */
  print::Begin();
  print::DeviceInfo();
  /* Init inceptors */

  /* Init sensors */
  airdata::Init();
  imu::Init();
  gnss::Init();
  /* Init VMS */

  /* Init effectors */

  /* Init datalog */

  /* Init telemetry */

  while (1) {
    /* Flush data to disk */

  }
}
