/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/airdata.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "ams5915/ams5915.h"
#include "statistics/statistics.h"

namespace airdata {
namespace {
/* AMS-5915 */
sensors::Ams5915 static_pres_(&STATIC_PRESS_I2C_BUS, STATIC_PRESS_ADDR, STATIC_PRESS_TRANSDUCER);
sensors::Ams5915 diff_pres_(&DIFF_PRESS_I2C_BUS, DIFF_PRESS_ADDR, DIFF_PRESS_TRANSDUCER);
/* Differential pressure statistics */
statistics::Running<float> diff_pres_stats_;
}  // anonymous

void Init() {
  print::Info("Initializing pressure transducers...");
  if (!static_pres_.Begin()) {
    print::Error("Unable to initialize communication with static pressure transducer.");
  }
  if (!diff_pres_.Begin()) {
    print::Error("Unable to initialize communication with differential pressure transducer.");
  }
  print::Info("done.\n");
}

void Calibrate() {
  if (diff_pres_.Read()) {
    diff_pres_stats_.Update(diff_pres_.pressure_pa());
  }
}

void Read(Airdata *ptr) {
  if (!ptr) {return;}
  if (static_pres_.Read()) {
    ptr->static_pres_pa = static_pres_.pressure_pa();
  }
  if (diff_pres_.Read()) {
    ptr->diff_pres_pa = diff_pres_.pressure_pa() - diff_pres_stats_.mean();
  }
}

}  // namespace airdata
