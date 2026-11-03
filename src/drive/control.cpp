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
  bool disableDrive = false;
  constexpr double AtoMV = 12000.0/127.0;
  double LY = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
  double RX = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_X));
  double LX = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
  double RY = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

  double strafe = 0;
  double fwd = 0;
  double turn = 0;

  double mv1 = fwd + strafe + turn;
  double mv2 = fwd - strafe - turn;
  double mv3 = fwd + strafe - turn;
  double mv4 = fwd - strafe + turn;
  double max = std::max({std::abs(mv1), std::abs(mv2), std::abs(mv3), std::abs(mv4)});
  if (max > 12000) {
    double mult = 12000.0 / max;
    mv1 *= mult;
    mv2 *= mult;
    mv3 *= mult;
    mv4 *= mult;
  }
  if (disableDrive) {mv1 = 0;mv2 = 0;mv3 = 0;mv4 = 0;}
  chassis.millivolts((int32_t)mv1, (int32_t)mv2, (int32_t)mv3, (int32_t)mv4);
}