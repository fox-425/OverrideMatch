#include "drive/control.h"
#include "utils.h"

Control::Control(Drive& chassis, Controller& primary):
  chassis(chassis), primary(primary)
{
  reversed = false;
}

void Control::Digitals() {
  /*
  bool A = primary.get_digital(E_CONTROLLER_DIGITAL_A);
  bool B = primary.get_digital(E_CONTROLLER_DIGITAL_B);
  bool X = primary.get_digital(E_CONTROLLER_DIGITAL_X);
  bool Y = primary.get_digital(E_CONTROLLER_DIGITAL_Y);
  bool N = primary.get_digital(E_CONTROLLER_DIGITAL_UP);
  bool E = primary.get_digital(E_CONTROLLER_DIGITAL_RIGHT);
  bool S = primary.get_digital(E_CONTROLLER_DIGITAL_DOWN);
  bool W = primary.get_digital(E_CONTROLLER_DIGITAL_LEFT);
  */

}

void Control::Arcade() {
  int32_t LY = primary.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
  int32_t RX = primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
  if (reversed) {LY = -LY;}

  LY = (float)LY * 94.48819f;
  RX = (float)RX * 94.48819f;
  float total = std::abs(LY) + std::abs(RX);

  if (total > 12000.0f) {
    float mult = 12000.0f / total;
    LY *= mult;
    RX *= mult;
  }

  float DL = LY + RX;
  float DR = LY - RX;

  chassis.millivolts(std::round(DL), std::round(DR));
}