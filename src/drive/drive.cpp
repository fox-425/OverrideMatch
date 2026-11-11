#include "drive/drive.h"
#include "data/EMSG.h"
// extern Motor MT;
extern EMSG eMsg;

MVs::MVs(int32_t MV1, int32_t MV2, int32_t MV3, int32_t MV4): MV1(MV1), MV2(MV2), MV3(MV3), MV4(MV4) {}

Drive::Drive(
  Motor& M1, Motor& M2, Motor& M3, Motor& M4,
  Motor& MC1, Motor& MC2, Motor& MC3, Motor& MT,
  adi::DigitalOut& claw, Optical& opt
):
  M1(M1), M2(M2), M3(M3), M4(M4),
  MC1(MC1), MC2(MC2), MC3(MC3), MT(MT),
  claw(claw), opt(opt),
  mvC1(0), mvC2(0), mvC3(0)
{}

void Drive::millivolts(int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4) {
  M1.move_voltage(mv1);
  M2.move_voltage(mv2);
  M3.move_voltage(mv3);
  M4.move_voltage(mv4);
}

void Drive::toggle(int32_t mv) {
  MT.move_voltage(mv);
}

void Drive::cascade(int32_t mv) {
  double vel = std::min({std::fabs(MC1.get_actual_velocity()), std::fabs(MC2.get_actual_velocity()), std::fabs(MC3.get_actual_velocity())});
  double cur = std::min({std::fabs(MC1.get_current_draw()), std::fabs(MC2.get_current_draw()), std::fabs(MC3.get_current_draw())});
  if (vel < 180 && cur > 1500) {
    MC1.brake();
    MC2.brake();
    MC3.brake();
    return;
  }
  MC1.move_voltage(mv);
  MC2.move_voltage(mv);
  MC3.move_voltage(mv);
}

void Drive::brake(motor_brake_mode_e_t type) {
  MotorBrake p1 = M1.get_brake_mode();
  MotorBrake p2 = M2.get_brake_mode();
  MotorBrake p3 = M3.get_brake_mode();
  MotorBrake p4 = M4.get_brake_mode();
  M1.set_brake_mode(type);
  M2.set_brake_mode(type);
  M3.set_brake_mode(type);
  M4.set_brake_mode(type);
  M1.brake();
  M2.brake();
  M3.brake();
  M4.brake();
  M1.set_brake_mode(p1);
  M2.set_brake_mode(p2);
  M3.set_brake_mode(p3);
  M4.set_brake_mode(p4);
}

void Drive::tare_position_all() {
  M1.tare_position();
  M2.tare_position();
  M3.tare_position();
  M4.tare_position();
}
