#pragma once
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include "pid.h"
using namespace pros;

struct Drive {
  Motor& M1;
  Motor& M2;
  Motor& M3;
  Motor& M4;
  Motor& MC1;
  Motor& MC2;
  Motor& MC3;
  Rotation& RC;
  PID diff;
  PID casH;
  int32_t mvC1;
  int32_t mvC2;
  int32_t mvC3;
  adi::DigitalOut &Intake1;
  adi::DigitalOut &Intake2;
  bool intake_state;

  Drive(
    Motor& M1, Motor& M2, Motor& M3, Motor& M4,
    Motor& MC1, Motor& MC2, Motor& MC3,
    Rotation& RC,
    adi::DigitalOut& Intake1, adi::DigitalOut& Intake2
  );

  void millivolts(int32_t mv1, int32_t mv2, int32_t mv3, int32_t mv4);
  void toggle_intake();
  void intake(int32_t mv);
  void cascade(int32_t mv);
  void setDiffTar(float tar);
  float getDiffTar();
  void initDiff();
  void diffLoop();

  void brake(motor_brake_mode_e_t type);

  void tare_position_all();
};
