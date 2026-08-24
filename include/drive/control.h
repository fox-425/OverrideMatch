#pragma once
#include "drive/drive.h"
#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

class Control {
  Drive& chassis;
  Controller& primary;
  bool reversed;

  Control(Drive& chassis, Controller& primary);

  void Digitals();

  void Arcade();
};