#pragma once
#include "pros/rtos.hpp"

struct EMSG {
  std::string content;
  pros::Mutex mutex;

  EMSG();

  void addText(std::string text);
  void clearText();
};