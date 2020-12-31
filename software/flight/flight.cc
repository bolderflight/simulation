/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "core/core.h"
#include "flight/global_defs.h"
#include "flight/hardware_defs.h"
#include "flight/print_msg.h"
#include "flight/inceptor.h"
#include "flight/airdata.h"
#include "flight/imu.h"
#include "flight/gnss.h"
#include "flight/effector.h"

/* Aircraft data */
AircraftData data;

/* Timer for sending effector commands */
IntervalTimer effector_timer;

/* Send effector commands to servos & motors */
void send_effector() {
  /* Stop the effector timer */
  effector_timer.end();
  /* Send effector commands to actuators */
  effector::Write();
}

/* IMU data ready interrupt */
void drdy() {
  /* Start the effector timer */
  effector_timer.begin(send_effector, EFFECTOR_DELAY_US);
  /* System time, us precision */
  data.sensor.sys_time_s = static_cast<double>(micros64()) / 1e6;
  /* Read inceptors */
  inceptor::Read(&data.inceptor);
  /* Read sensors */
  airdata::Read(&data.sensor.airdata);
  imu::Read(&data.sensor.imu);
  gnss::Read(&data.sensor.gnss);
  /* Run the VMS */

  /* Effector commands */
  effector::Cmd(data.effector);
  /* Datalog */
  // datalog::Write(data);

  /* Telemetry */

}

int main() {
  /* Init debug messages */
  print::Begin();
  print::DeviceInfo();
  /* Init inceptors */
  inceptor::Init();
  /* Init sensors */
  airdata::Init();
  imu::Init();
  gnss::Init();
  /* Init VMS */

  /* Init effectors */
  effector::Init();
  /* Init datalog */
  // datalog::Init();
  /* Init telemetry */

  /* Attach IMU data ready callback */
  imu::AttachCallback(drdy);
  while (1) {
    /* Flush datalog to SD */
    // datalog::Flush();
  }
}
