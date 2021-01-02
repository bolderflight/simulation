/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_GLOBAL_DEFS_H_
#define INCLUDE_FLIGHT_GLOBAL_DEFS_H_

#include <cinttypes>
#include "flight/hardware_defs.h"

/* IMU data */
struct ImuData {
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
  float mag_x_ut;
  float mag_y_ut;
  float mag_z_ut;
};

/* Airdata */
struct Airdata {
  float static_pres_pa;
  float diff_pres_pa;
};

/* GNSS data */
struct GnssData {
  int8_t fix;
  int8_t num_sats;
  int16_t week;
  float alt_wgs84_m;
  float ned_vel_x_mps;
  float ned_vel_y_mps;
  float ned_vel_z_mps;
  float horz_acc_m;
  float vert_acc_m;
  float vel_acc_mps;
  double lat_rad;
  double lon_rad;
  double tow_s;
};

/* Sensor data */
struct SensorData {
  double sys_time_s;
  ImuData imu;
  GnssData gnss;
  Airdata airdata;
};

/* Pilot inputs */
struct InceptorData {
  int8_t mode0;
  int8_t mode1;
  int8_t mode2;
  bool throttle_en;
  float throttle;
  float pitch;
  float roll;
  float yaw;
};

/* Sensor processing data */
struct SenProcData {
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
  float pitch_rad;
  float roll_rad;
  float yaw_rad;
  float static_pres_pa;
  float diff_pres_pa;
  float alt_wgs84_m;
  float alt_pres_m;
  float ias_mps;
  float eas_mps;
  float ned_vel_x_mps;
  float ned_vel_y_mps;
  float ned_vel_z_mps;
  double latitude_rad;
  double longitude_rad;
};

/* Effector commands */
struct EffectorCmds {
  std::array<float, NUM_PWM_PINS> pwm;
  std::array<float, 16> sbus;
};

/* Aux data */
struct AuxData {
  std::array<float, 24> data;
};

/* Aircraft data */
struct AircraftData {
  InceptorData inceptor;
  SensorData sensor;
  SenProcData sen_proc;
  EffectorCmds effector;
  AuxData aux;
};

#endif  // INCLUDE_FLIGHT_GLOBAL_DEFS_H_
