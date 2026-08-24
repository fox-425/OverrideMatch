#pragma once
#include "pros/motors.hpp"
using namespace pros;

class Drive {
  Motor& M1;
  Motor& M2;
  Motor& M3;
  Motor& M4;
  Motor& M5;
  Motor& M6;

public:
  Drive(
    Motor& M1, Motor& M2, Motor& M3, Motor& M4, Motor& M5, Motor& M6
  );

  void millivolts(int32_t mvL, int32_t mvR);

  void brake(motor_brake_mode_e_t type);

  void tare_position_all();

  float getPosition(int32_t sel);
};