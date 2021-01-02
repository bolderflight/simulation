/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "flight/effector.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "sbus/sbus.h"
#include "pwm/pwm.h"
#include "polytools/polytools.h"

namespace effector {
namespace {
/* SBUS object */
actuators::Sbus sbus_(&SBUS_UART);
/* PWM object */
actuators::Pwm<NUM_PWM_PINS> pwm_(PWM_PINS);
/* SBUS commands */
std::array<uint16_t, 16> sbus_cmds;
/* PWM commands */
std::array<uint16_t, NUM_PWM_PINS> pwm_cmds;
/* Maximum number of polynomial coefficients */
static constexpr int MAX_COEF = 5;
/* SBUS angle command to PWM polynomial coefficients */
float SBUS_COEF[16][MAX_COEF] = {
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}
};
std::size_t SBUS_COEF_LEN[16] = {
0
};
/* PWM angle command to PWM polynomial coefficients */
float PWM_COEF[NUM_PWM_PINS][MAX_COEF] = {
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0}
};
std::size_t PWM_COEF_LEN[NUM_PWM_PINS] = {
0
};
}  // anonymous

void Init() {
  print::Info("Initializing effectors...");
  sbus_.Begin();
  pwm_.Begin();
  print::Info("done.\n");
}
void Cmd(const EffectorCmds &ref) {
  for (std::size_t i = 0; i < ref.sbus.size(); i++) {
    sbus_cmds[i] = static_cast<uint16_t>(polytools::polyval(SBUS_COEF[i], SBUS_COEF_LEN[i], ref.sbus[i]));
  }
  for (std::size_t i = 0; i < ref.pwm.size(); i++) {
    pwm_cmds[i] = static_cast<uint16_t>(polytools::polyval(PWM_COEF[i], PWM_COEF_LEN[i], ref.pwm[i]));
  }
  sbus_.tx_channels(sbus_cmds);
  pwm_.tx_channels(pwm_cmds);
}
void Write() {
  sbus_.Write();
  pwm_.Write();
}

}  // namespace inceptor
