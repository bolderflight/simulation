/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include <array>
#include "flight/inceptor.h"
#include "flight/print_msg.h"
#include "flight/hardware_defs.h"
#include "flight/global_defs.h"
#include "sbus/sbus.h"
#include "polytools/polytools.h"

namespace inceptor {
namespace {
/* SBUS object */
sensors::Sbus sbus_(&SBUS_UART);
/* SBUS data */
std::array<uint16_t, 16> sbus_data;
/* Polyval coefficients for attitude cmd, map SBUS to -1 to +1 */
static constexpr std::array<float, 2> ATT_CMD_POLY_COEF_ = {0.001220256253813301f, -1.209884075655888f};
/* Polyval coefficients for throttle, map SBUS to 0 to +1 */
static constexpr std::array<float, 2> THR_CMD_POLY_COEF_ = {0.0006101281269066503f, -0.1049420378279439f};
/* Polyval coefficients for mode switches, map SBUS to 0 to +1 */
static constexpr std::array<float, 2> MODE_POLY_COEF_ = {0.0006101281269066503f,  -0.1049420378279439f};

}  // anonymous

void Init() {
  print::Info("Initializing inceptors...");
  sbus_.Begin();
  print::Info("done.\n");
}

void Read(InceptorData *ptr) {
  if (sbus_.Read()) {
    sbus_data = sbus_.rx_channels();
    /* Throttle, roll, pitch, yaw */
    ptr->throttle = polytools::polyval(THR_CMD_POLY_COEF_, static_cast<float>(sbus_data[0]));
    ptr->roll = polytools::polyval(ATT_CMD_POLY_COEF_, static_cast<float>(sbus_data[1]));
    ptr->pitch = polytools::polyval(ATT_CMD_POLY_COEF_, static_cast<float>(sbus_data[2]));
    ptr->yaw = polytools::polyval(ATT_CMD_POLY_COEF_, static_cast<float>(sbus_data[3]));
    /* Mode0 */
    if (polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[4])) < 0.3f) {
      ptr->mode0 = 2;
    } else if (polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[4])) < 0.6f) {
      ptr->mode0 = 1;
    } else {
      ptr->mode0 = 0;
    }
    /* Mode1 */
    if (polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[5])) < 0.3f) {
      ptr->mode1 = 2;
    } else if (polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[5])) < 0.6f) {
      ptr->mode1 = 1;
    } else {
      ptr->mode1 = 0;
    }
    /* Mode2 */
    if (polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[6])) < 0.3f) {
      ptr->mode2 = 2;
    } else if (polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[6])) < 0.6f) {
      ptr->mode2 = 1;
    } else {
      ptr->mode2 = 0;
    }
    /* Throttle safety */
    ptr->throttle_en = polytools::polyval(MODE_POLY_COEF_, static_cast<float>(sbus_data[7])) > 0.5f;
  }
}

}  // namespace inceptor
