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
  bool R1 = primary.get_digital(E_CONTROLLER_DIGITAL_R1);
}

void Control::Arcade() {
  int32_t LY = primary.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
  int32_t RX = primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
  if (reversed) {LY = -LY;}
  
  LY = (float)LY * 94.4882f;
  RX = (float)RX * 94.4882f;
  if (LY > 12000) {
    LY = 12000;
  } else if (LY < -12000) {
    LY = -12000;
  }
  if (RX > 12000) {
    RX = 12000;
  } else if (RX < -12000) {
    RX = -12000;
  }
  
  if (LY > 84.667f) {
    LY = 2.0*(float)LY - 127;
  } else if (LY < -84.667f) {
    LY = 2.0*(float)LY + 127;
  } else {
    LY = 0.5*(float)LY;
  }

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