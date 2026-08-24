#include "drive/drive.h"

Drive::Drive(Motor& M1, Motor& M2, Motor& M3, Motor& M4, Motor& M5, Motor& M6):
  M1(M1), M2(M2), M3(M3), M4(M4), M5(M5), M6(M6)
{}

void Drive::millivolts(int32_t mvL, int32_t mvR) {
  M1.move_voltage(mvL);
  M2.move_voltage(mvL);
  M3.move_voltage(mvL);
  M4.move_voltage(mvR);
  M5.move_voltage(mvR);
  M6.move_voltage(mvR);
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