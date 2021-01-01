/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/gnss.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "ublox/ublox.h"

namespace gnss {
namespace {
/* uBlox GNSS */
sensors::Ublox gnss_(&GNSS_UART);
/* Year, Month, Day to GNSS week */
int GnssWeek(int year, int month, int day) {
  static const int month_day[2][12] = {
    {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
    {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}
  };
  static const int JAN61980 = 44244;
  static const int JAN11901 = 15385;
  int yday, mjd, leap;
  leap = (year % 4 == 0);
  yday = month_day[leap][month - 1] + day;
  mjd = ((year - 1901) / 4) * 1461 + ((year - 1901) % 4) * 365 + yday - 1 + JAN11901;
  return (mjd - JAN61980) / 7;
}
}  // anonymous

void Init() {
  print::Info("Initializing GNSS...");
  /* Initialize communication */
  if (!gnss_.Begin(GNSS_BAUD)) {
    print::Error("Unable to initialize communication with GNSS receiver.");
  }
  print::Info("done.\n");
}

void Read(GnssData *ptr) {
  if (!ptr) {return;}
  if (gnss_.Read()) {
    ptr->fix = static_cast<int8_t>(gnss_.fix());
    ptr->num_sats = gnss_.num_satellites();
    ptr->tow_s = static_cast<double>(gnss_.tow_ms()) / 1e3;
    ptr->week = GnssWeek(gnss_.year(), gnss_.month(), gnss_.day());
    ptr->lat_rad = gnss_.lat_rad();
    ptr->lon_rad = gnss_.lon_rad();
    ptr->alt_wgs84_m = gnss_.alt_wgs84_m();
    ptr->ned_vel_x_mps = gnss_.north_velocity_mps();
    ptr->ned_vel_y_mps = gnss_.east_velocity_mps();
    ptr->ned_vel_z_mps = gnss_.down_velocity_mps();
    ptr->horz_acc_m = gnss_.horizontal_accuracy_m();
    ptr->vert_acc_m = gnss_.vertical_accuracy_m();
    ptr->vel_acc_mps = gnss_.velocity_accuracy_mps();
  }
}

}  // namespace gnss
