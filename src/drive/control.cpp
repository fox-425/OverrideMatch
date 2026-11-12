#include "drive/control.h"
#include "utils.h"
#include "data/EMSG.h"
#include "tracking/tracker.h"
extern EMSG eMsg;
extern Tracker tracker;

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
  bool Y = primary.get_digital(E_CONTROLLER_DIGITAL_Y);
  bool R1_prev = prev & 0b00000001;
  bool R2_prev = prev & 0b00000010;

  if (R1 && R2) {}
  else if (R1 && !R1_prev) {
    chassis.cascade(12000);
  } else if (R2 && !R2_prev) {
    chassis.cascade(-12000);
  } else if (!R1 && !R2 && (R1_prev || R2_prev)) {
    chassis.cascade(0);
  }
  chassis.toggle(Y ? 12000 : 0);

  chassis.toggleToColor();

  uint8_t next = (R1 ? 0b00000001 : 0) | (R2 ? 0b00000010 : 0);
  return next;
}

void Control::Analog(MVs& pMVs, PID& pidH) {
  constexpr bool disableDrive = false;
  constexpr double slew = 2000.0;
  constexpr double AtoMV = 12000.0/127.0;
  double LY = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
  double RX = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_X));
  double LX = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_LEFT_X));
  // double RY = AtoMV * (double)(primary.get_analog(E_CONTROLLER_ANALOG_RIGHT_Y));

  double fwd = LY;
  double strafe = LX;
  double turn = 0;

  pidH.setTarget(RX*0.75);
  turn = pidH.update(pros::millis(), tracker.imu.get_gyro_rate().z).out;
  if (fabs(turn) < 60) {turn = 0;}
  // turn = RX;
  // eMsg.clearText();
  // eMsg.addText(std::format("in/s: {:.2f}, fwd: {:.2f}, in: {:.2f}", tracker.odom.dy*100.0, fwd, LY*85.0/12000.0));

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
  mv1 = pMVs.MV1 + std::clamp(mv1 - pMVs.MV1, -slew, slew);
  mv2 = pMVs.MV2 + std::clamp(mv2 - pMVs.MV2, -slew, slew);
  mv3 = pMVs.MV3 + std::clamp(mv3 - pMVs.MV3, -slew, slew);
  mv4 = pMVs.MV4 + std::clamp(mv4 - pMVs.MV4, -slew, slew);

  if (disableDrive) {mv1 = 0;mv2 = 0;mv3 = 0;mv4 = 0;}
  pMVs = MVs((int32_t)mv1, (int32_t)mv2, (int32_t)mv3, (int32_t)mv4);
  chassis.millivolts((int32_t)mv1, (int32_t)mv2, (int32_t)mv3, (int32_t)mv4);

  // chassis.cascade((int32_t)RY);
}