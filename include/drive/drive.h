#pragma once
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pid.h"
using namespace pros;

class Drive {
  Motor& M1;
  Motor& M2;
  Motor& M3;
  Motor& M4;
  Motor& M5;
  Motor& M6;
  Motor& MI;
  Motor& MC1;
  Motor& MC2;
  Rotation& RC;
  PID diff;
  int32_t mvC1;
  int32_t mvC2;
  adi::DigitalOut &Intake1;
  adi::DigitalOut &Intake2;
  bool intake_state;

public:
  Drive(
    Motor& M1, Motor& M2, Motor& M3, Motor& M4, Motor& M5, Motor& M6,
    Motor& MI, Motor& MC1, Motor& MC2,
    Rotation& RC,
    adi::DigitalOut& Intake1, adi::DigitalOut& Intake2
  );

  void millivolts(int32_t mvL, int32_t mvR);
  void toggle_intake();
  void intake(int32_t mv);
  void cascade(int32_t mv1, int32_t mv2);
  void setDiffTar(float tar);
  float getDiffTar();
  void initDiff();
  void diffLoop();

  void brake(motor_brake_mode_e_t type);

  void tare_position_all();

  float getPosition(int32_t sel);
};
