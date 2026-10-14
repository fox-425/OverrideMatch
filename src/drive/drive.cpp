#include "drive/drive.h"
#include "data/EMSG.h"
extern Motor MT;
extern EMSG eMsg;

Drive::Drive(
  Motor& M1, Motor& M2, Motor& M3, Motor& M4, Motor& M5, Motor& M6,
  Motor& MI, Motor& MC1, Motor& MC2,
  Rotation& RC,
  adi::DigitalOut& Intake1, adi::DigitalOut& Intake2
):
  M1(M1), M2(M2), M3(M3), M4(M4), M5(M5), M6(M6),
  MI(MI), MC1(MC1), MC2(MC2),
  RC(RC), diff(0.0, 40.0, 1.0, 60.0, false), mvC1(0), mvC2(0),
  Intake1(Intake1), Intake2(Intake2), intake_state(false)
{
  diff.setExit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  diff.setIntegral(0.0f, 0.0f);
}

void Drive::millivolts(int32_t mvL, int32_t mvR) {
  M1.move_voltage(mvL);
  M2.move_voltage(mvL);
  M3.move_voltage(mvL);
  M4.move_voltage(mvR);
  M5.move_voltage(mvR);
  M6.move_voltage(mvR);
}

void Drive::toggle_intake() {
  intake_state = !intake_state;
  Intake1.set_value(intake_state);
  Intake2.set_value(intake_state);
}

void Drive::intake(int32_t mv) {
  MI.move_voltage(mv);
}

void Drive::cascade(int32_t mv1, int32_t mv2) {
  mvC1 = mv1;
  mvC2 = mv2;
}

void Drive::setDiffTar(float tar) {
  diff.setTarget(tar);
}

float Drive::getDiffTar() {
  return diff.getTarget();
}

void Drive::initDiff() {
  RC.set_position(0);
  MT.set_encoder_units(MotorUnits::degrees);
  MT.tare_position();
  pros::Task diffLooping([this]() {
    this->diffLoop();
  });
}

void Drive::diffLoop() {
  while(true) {
    // float diffVal = diff.update(pros::millis(), (float)(RC.get_position()) / 100.0f).out;
    float diffVal = diff.update(pros::millis(), MT.get_position()).out;
    eMsg.clearText();
    eMsg.addText(std::format("{:.2f}", diff.getTarget()) + " " + std::format("{:.2f}", MT.get_position()) + " " + std::format("{:.2f}", diffVal));
    MT.move_voltage(diffVal);
    // MC1.move_voltage(mvC1 + diffVal);
    // MC2.move_voltage(mvC2 - diffVal);
    pros::delay(10);
  }
}

void Drive::brake(motor_brake_mode_e_t type) {
  M1.set_brake_mode(type);
  M2.set_brake_mode(type);
  M3.set_brake_mode(type);
  M4.set_brake_mode(type);
  M5.set_brake_mode(type);
  M6.set_brake_mode(type);
}

void Drive::tare_position_all() {
  M1.tare_position();
  M2.tare_position();
  M3.tare_position();
  M4.tare_position();
  M5.tare_position();
  M6.tare_position();
}

float Drive::getPosition(int32_t sel) {
  if (sel == -1) {
    return static_cast<float>((M1.get_position() + M2.get_position() + M3.get_position()) / 3.0);
  } else if (sel == 1) {
    return static_cast<float>((M4.get_position() + M5.get_position() + M6.get_position()) / 3.0);
  } else {
    return static_cast<float>((M1.get_position() + M2.get_position() + M3.get_position() + M4.get_position() + M5.get_position() + M6.get_position()) / 6.0);
  }
}
