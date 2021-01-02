/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems
*/

#include "flight/vms.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "autocode_vms.h"

namespace vms {
namespace {
/* VMS class */
Vms vms_;
}  // anonymous

void Init() {
  print::Info("Initializing VMS...");
  vms_.initialize();
  print::Info("done.\n");
}

void Run(const InceptorData &inceptor, const SensorData &sensor, SenProcData *sen_proc, AuxData *aux, EffectorCmds *effector) {
  vms_.run(inceptor, sensor, sen_proc, aux, effector);
}

}  // namespace vms
