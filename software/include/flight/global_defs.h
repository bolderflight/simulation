/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#ifndef INCLUDE_FLIGHT_GLOBAL_DEFS_H_
#define INCLUDE_FLIGHT_GLOBAL_DEFS_H_

#include <cinttypes>

/* 6DoF IMU data */
struct ImuData {
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_radps;
  float gyro_y_radps;
  float gyro_z_radps;
};

/* Magnetometer data */
struct MagData {
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
  ImuData imu;
  MagData mag;
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
  float alt_wgs84_m;
  float alt_pres_m;
  float ned_vel_x_mps;
  float ned_vel_y_mps;
  float ned_vel_z_mps;
  double latitude_rad;
  double longitude_rad;
};

/* Effector commands */
struct EffectorCmds {
  float pwm[8];
  float sbus[16];
};

/* Aircraft data */
struct AircraftData {
  InceptorData inceptor;
  SensorData sensor;
  SenProcData sen_proc;
  EffectorCmds effector;
  float aux[24];
};

#endif  // INCLUDE_FLIGHT_GLOBAL_DEFS_H_
