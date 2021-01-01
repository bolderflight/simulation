/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/datalog.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "logger/logger.h"
#include "framing/framing.h"
#include "datalog.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

namespace datalog {
namespace {
/* Datalog file name */
std::string DATA_LOG_NAME_ = "flight_data";
/* Datalog message from protobuf */
DatalogMessage datalog_msg_;
/* SD card */
SdFatSdioEX sd_;
/* Logger object */
Logger<400> logger_(&sd_);
/* Framing */
framing::Encoder<DatalogMessage_size> encoder;
/* nanopb buffer for encoding */
uint8_t data_buffer_[DatalogMessage_size];
pb_ostream_t stream_;
}  // anonymous

void Init() {
  print::Info("Initializing datalog...");  
  /* Initialize SD card */
  sd_.begin();
  /* Initialize logger */
  int file_num = logger_.Init(DATA_LOG_NAME_);
  if (file_num < 0) {
    print::Error("Unable to initialize datalog.");
  }
  print::Info("done. Created datalog file ");
  print::Info(DATA_LOG_NAME_);
  print::Info(std::to_string(file_num));
  print::Info(".bfs\n");
  return;
}
void Write(const AircraftData &ref) {
  /* Fill the datalog message */
  datalog_msg_.inceptor_mode0 = ref.inceptor.mode0;
  datalog_msg_.inceptor_mode1 = ref.inceptor.mode1;
  datalog_msg_.inceptor_mode2 = ref.inceptor.mode2;
  datalog_msg_.inceptor_throttle_en = ref.inceptor.throttle_en;
  datalog_msg_.inceptor_throttle = ref.inceptor.throttle;
  datalog_msg_.inceptor_pitch = ref.inceptor.pitch;
  datalog_msg_.inceptor_roll = ref.inceptor.roll;
  datalog_msg_.inceptor_yaw = ref.inceptor.yaw;
  datalog_msg_.sys_time_s = ref.sensor.sys_time_s;
  datalog_msg_.imu_accel_x_mps2 = ref.sensor.imu.accel_x_mps2;
  datalog_msg_.imu_accel_y_mps2 = ref.sensor.imu.accel_y_mps2;
  datalog_msg_.imu_accel_z_mps2 = ref.sensor.imu.accel_z_mps2;
  datalog_msg_.imu_gyro_x_radps = ref.sensor.imu.gyro_x_radps;
  datalog_msg_.imu_gyro_y_radps = ref.sensor.imu.gyro_y_radps;
  datalog_msg_.imu_gyro_z_radps = ref.sensor.imu.gyro_z_radps;
  datalog_msg_.imu_mag_x_ut = ref.sensor.imu.mag_x_ut;
  datalog_msg_.imu_mag_y_ut = ref.sensor.imu.mag_y_ut;
  datalog_msg_.imu_mag_z_ut = ref.sensor.imu.mag_z_ut;
  datalog_msg_.gnss_fix = ref.sensor.gnss.fix;
  datalog_msg_.gnss_num_sats = ref.sensor.gnss.num_sats;
  datalog_msg_.gnss_week = ref.sensor.gnss.week;
  datalog_msg_.gnss_alt_wgs84_m = ref.sensor.gnss.alt_wgs84_m;
  datalog_msg_.gnss_ned_vel_x_mps = ref.sensor.gnss.ned_vel_x_mps;
  datalog_msg_.gnss_ned_vel_y_mps = ref.sensor.gnss.ned_vel_y_mps;
  datalog_msg_.gnss_ned_vel_z_mps = ref.sensor.gnss.ned_vel_z_mps;
  datalog_msg_.gnss_horz_acc_m = ref.sensor.gnss.horz_acc_m;
  datalog_msg_.gnss_vert_acc_m = ref.sensor.gnss.vert_acc_m;
  datalog_msg_.gnss_vel_acc_mps = ref.sensor.gnss.vel_acc_mps;
  datalog_msg_.gnss_lat_rad = ref.sensor.gnss.lat_rad;
  datalog_msg_.gnss_lon_rad = ref.sensor.gnss.lon_rad;
  datalog_msg_.gnss_tow_s = ref.sensor.gnss.tow_s;
  datalog_msg_.airdata_static_pres_pa = ref.sensor.airdata.static_pres_pa;
  datalog_msg_.airdata_diff_pres_pa = ref.sensor.airdata.diff_pres_pa;
  datalog_msg_.sen_proc_accel_x_mps2 = ref.sen_proc.accel_x_mps2;
  datalog_msg_.sen_proc_accel_y_mps2 = ref.sen_proc.accel_y_mps2;
  datalog_msg_.sen_proc_accel_z_mps2 = ref.sen_proc.accel_z_mps2;
  datalog_msg_.sen_proc_gyro_x_radps = ref.sen_proc.gyro_x_radps;
  datalog_msg_.sen_proc_gyro_y_radps = ref.sen_proc.gyro_y_radps;
  datalog_msg_.sen_proc_gyro_z_radps = ref.sen_proc.gyro_z_radps;
  datalog_msg_.sen_proc_pitch_rad = ref.sen_proc.pitch_rad;
  datalog_msg_.sen_proc_roll_rad = ref.sen_proc.roll_rad;
  datalog_msg_.sen_proc_yaw_rad = ref.sen_proc.yaw_rad;
  datalog_msg_.sen_proc_alt_wgs84_m = ref.sen_proc.alt_wgs84_m;
  datalog_msg_.sen_proc_alt_pres_m = ref.sen_proc.alt_pres_m;
  datalog_msg_.sen_proc_ned_vel_x_mps = ref.sen_proc.ned_vel_x_mps;
  datalog_msg_.sen_proc_ned_vel_y_mps = ref.sen_proc.ned_vel_y_mps;
  datalog_msg_.sen_proc_ned_vel_z_mps = ref.sen_proc.ned_vel_z_mps;
  datalog_msg_.sen_proc_latitude_rad = ref.sen_proc.latitude_rad;
  datalog_msg_.sen_proc_longitude_rad = ref.sen_proc.longitude_rad;
  for (std::size_t i = 0; i < ref.effector.pwm.size(); i++) {
    datalog_msg_.effector_pwm[i] = ref.effector.pwm[i];
  }
  for (std::size_t i = 0; i < ref.effector.sbus.size(); i++) {
    datalog_msg_.effector_sbus[i] = ref.effector.sbus[i];
  }
  for (std::size_t i = 0; i < ref.aux.size(); i++) {
    datalog_msg_.aux[i] = ref.aux[i];
  }
  /* Encode */
  stream_ = pb_ostream_from_buffer(data_buffer_, sizeof(data_buffer_));
  if (!pb_encode(&stream_, DatalogMessage_fields, &datalog_msg_)) {
    print::Warning("Error encoding datalog.");
    return;
  }
  std::size_t msg_len = stream_.bytes_written;
  /* Framing */
  std::size_t bytes_written = encoder.Write(data_buffer_, msg_len);
  if (msg_len != bytes_written) {
    print::Warning("Error framing datalog.");
    return;
  }
  /* Write the data */
  logger_.Write(encoder.Data(), encoder.Size());
}
void Flush() {
  logger_.Flush();
}
void Close() {
  logger_.Close();
}

}  // namespace datalog
