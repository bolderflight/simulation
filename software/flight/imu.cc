/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/imu.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "mpu9250/mpu9250.h"
#include "statistics/statistics.h"

namespace imu {
namespace {
/* Mag calibration data */
float hx_bias = 0;
float hy_bias = 0;
float hz_bias = 0;
float hx_scale = 1;
float hy_scale = 1;
float hz_scale = 1;
/* Accel calibration data */
float ax_bias = 0;
float ay_bias = 0;
float az_bias = 0;
float ax_scale = 1;
float ay_scale = 1;
float az_scale = 1;
/* MPU-9250 IMU */
sensors::Mpu9250 imu_(&IMU_SPI_BUS, IMU_CS);
static constexpr sensors::Mpu9250::AccelRange ACCEL_RANGE_ = sensors::Mpu9250::ACCEL_RANGE_16G;
static constexpr sensors::Mpu9250::GyroRange Gyro_RANGE_ = sensors::Mpu9250::GYRO_RANGE_2000DPS;
static constexpr sensors::Mpu9250::DlpfBandwidth DLPF_ = sensors::Mpu9250::DLPF_BANDWIDTH_20HZ;
static constexpr unsigned int SRD_ = 1000 / FRAME_RATE_HZ - 1;
static Eigen::Matrix3f ROTATION_ = (Eigen::Matrix3f() << 0, 1, 0, -1, 0, 0, 0, 0, 1).finished();
/* IMU statistics */
statistics::Running<float> gx_stats_, gy_stats_, gz_stats_;
}  // anonymous

void Init() {
  /* Init IMU */
  print::Info("Initializing IMU...");
  /* Initialize communication */
  if (!imu_.Begin()) {
    print::Error("Unable to initialize communication with IMU.");
  }
  /* Set the rotation */
  imu_.rotation(ROTATION_);
  /* Set the accel range */
  if (!imu_.accel_range(ACCEL_RANGE_)) {
    print::Error("Unable to set IMU accel full-scale range.");
  }
  /* Set the gryo range */
  if (!imu_.gyro_range(Gyro_RANGE_)) {
    print::Error("Unable to set IMU gyro full-scale range.");
  }
  /* Set the DLPF */
  if (!imu_.dlpf_bandwidth(DLPF_)) {
    print::Error("Unable to set IMU DLPF bandwidth.");
  }
  /* Set the SRD */
  if (!imu_.sample_rate_divider(SRD_)) {
    print::Error("Unable to set IMU sample rate divider.");
  }
  /* Enable data ready interrupt */
  if (!imu_.EnableDrdyInt()) {
    print::Error("Unable to set data ready interrupt.");
  }
  print::Info("done.\n");
}

void Calibrate() {
  if (imu_.Read()) {
    gx_stats_.Update(imu_.gyro_x_radps());
    gy_stats_.Update(imu_.gyro_y_radps());
    gz_stats_.Update(imu_.gyro_z_radps());
  }
}

void AttachCallback(void (*function)()) {
  imu_.DrdyCallback(IMU_DRDY, function);
}

void Read(ImuData *ptr) {
  if (!ptr) {return;}
  if (imu_.Read()) {
    ptr->accel_x_mps2 = (imu_.accel_x_mps2() - ax_bias) * ax_scale;
    ptr->accel_y_mps2 = (imu_.accel_y_mps2() - ay_bias) * ay_scale;
    ptr->accel_z_mps2 = (imu_.accel_z_mps2() - az_bias) * az_scale;
    ptr->gyro_x_radps = imu_.gyro_x_radps() - gx_stats_.mean();
    ptr->gyro_y_radps = imu_.gyro_y_radps() - gy_stats_.mean();
    ptr->gyro_z_radps = imu_.gyro_z_radps() - gz_stats_.mean();
    ptr->mag_x_ut = (imu_.mag_x_ut() - hx_bias) * hx_scale;
    ptr->mag_y_ut = (imu_.mag_y_ut() - hy_bias) * hy_scale;
    ptr->mag_z_ut = (imu_.mag_z_ut() - hz_bias) * hz_scale;
  }
}

}  // namespace imu
