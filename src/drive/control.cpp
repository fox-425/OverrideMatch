#include "drive/control.h"
#include "utils.h"

Control::Control(Drive& chassis, Controller& primary):
  chassis(chassis), primary(primary)
{}

uint8_t Control::Buttons(uint8_t prev) {
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
  bool R2 = primary.get_digital(E_CONTROLLER_DIGITAL_R2);
  bool R1_prev = prev & 0b00000001;

  if (R1 && !R1_prev) {
    chassis.toggle_intake();
  }
  if (R2) {
    chassis.intake(12000);
  } else {
    chassis.intake(0);
  }

  uint8_t next = R1 ? 0b00000001 : 0;
  return next;
}

void Control::Analog() {
  constexpr double AtoMV = 12000.0/127.0;
  int32_t LY = AtoMV * (double)(-primary.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
  int32_t RX = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_X));
  int32_t LX = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
  int32_t RY = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

  if (LY > 254.0/3.0) {LY = 2*LY - 127;}
  else if (LY < -84.667f) {LY = 2*LY + 127;}
  else {LY /= 2;}
  
  int32_t total = std::abs(LY) + std::abs(RX);
  if (total > 12000) {
    double mult = 12000.0 / (double)total;
    LY *= mult;
    RX *= mult;
  }
  double DL = LY + RX;
  double DR = LY - RX;
  chassis.millivolts(std::round(DL), std::round(DR));

  total = std::abs(LX) + std::abs(RY);
  if (total > 12000) {
    double mult = 12000.0/ (double)total;
    LX *= mult;
    RY *= mult;
  }
  double C1 = LX + RY;
  double C2 = LX - RY;
  chassis.cascade(std::round(C1), std::cascade(C2));
}
