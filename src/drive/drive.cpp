#include "drive/drive.h"
#include "data/EMSG.h"
// extern Motor MT;
extern EMSG eMsg;

Drive::Drive(
  Motor& M1, Motor& M2, Motor& M3, Motor& M4,
  Motor& MC1, Motor& MC2, Motor& MC3,
  Rotation& RC,
  adi::DigitalOut& Intake1, adi::DigitalOut& Intake2
):
  M1(M1), M2(M2), M3(M3), M4(M4),
  MC1(MC1), MC2(MC2), MC3(MC3),
  RC(RC), diff(0.0, 0.025, 0.01, 0.10, false), casH(0.0, 0.05, 0.02, 0.20, false), mvC1(0), mvC2(0), mvC3(0),
  Intake1(Intake1), Intake2(Intake2), intake_state(false)
{
  diff.setExit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  diff.setIntegral(0.0f, 0.0f);
  casH.setExit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  casH.setIntegral(0.0f, 0.0f);
}

void Drive::millivolts(int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4) {
  M1.move_voltage(mv1);
  M2.move_voltage(mv2);
  M3.move_voltage(mv3);
  M4.move_voltage(mv4);
}

void Drive::toggle_intake() {
  intake_state = !intake_state;
  Intake1.set_value(intake_state);
  Intake2.set_value(intake_state);
}

void Drive::intake(int32_t mv) {
  // MI.move_voltage(mv);
}

void Drive::cascade(int32_t mv) {
  mvC1 = mv;
  mvC2 = mv;
  mvC3 = mv;
}

void Drive::setDiffTar(float tar) {
  diff.setTarget(tar);
}

float Drive::getDiffTar() {
  return diff.getTarget();
}

void Drive::initDiff() {
  RC.set_position(0);
  // MT.set_encoder_units(MotorUnits::degrees);
  // MT.tare_position();
  pros::Task diffLooping([this]() {
    this->diffLoop();
  });
}

void Drive::diffLoop() {
  while(true) {
    // PID::PIDoutput diffVal = diff.update(pros::millis(), (float)(RC.get_position()) / 100.0f);
    // eMsg.clearText();
    // eMsg.addText(std::format("{:.2f}", diff.getTarget()) + " " + std::format("{:.2f}", (double)RC.get_position() / 100.0) + " " + std::format("{:.2f}", diffVal.out) + " " + std::format("{:.2f}", diffVal.p) + " " + std::format("{:.2f}", diffVal.i) + " " + std::format("{:.2f}", diffVal.d));
    
    // if (mvC1 == 0 && mvC2 == 0) {
    //   casH.setTarget()
    // }
    // MC1.move_voltage(mvC1 + diffVal.out*1000.0f);
    // MC2.move_voltage(mvC2 - diffVal.out*1000.0f);
    MC1.move_voltage(mvC1);
    MC2.move_voltage(mvC2);
    MC3.move_voltage(mvC3);
    pros::delay(10);
  }
}

void Drive::brake(motor_brake_mode_e_t type) {
  M1.set_brake_mode(type);
  M2.set_brake_mode(type);
  M3.set_brake_mode(type);
  M4.set_brake_mode(type);
}

void Drive::tare_position_all() {
  M1.tare_position();
  M2.tare_position();
  M3.tare_position();
  M4.tare_position();
}
