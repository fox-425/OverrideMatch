#include "data/screenWriting.h"
#include "data/EMSG.h"
#include "pros/screen.hpp"
#include "pros/motors.hpp"

extern EMSG eMsg;
extern pros::Motor M1;
extern pros::Motor M2;
extern pros::Motor M3;
extern pros::Motor M4;
extern pros::Motor M5;
extern pros::Motor M6;
extern pros::Motor MI;

namespace Printing {
  void init() {
    pros::screen::set_eraser(0x00000000);
    pros::screen::set_pen(0x00FFFFFF);

    pros::Task screenTask(loop, 7);
  }

  void loop() {
    while (true) {
      pros::screen::erase();
      update();
      pros::delay(100);
    }
  }

  void update() {
    celMI = (uint8_t)MI.get_temperature();
    celM1 = (uint8_t)M1.get_temperature();
    celM2 = (uint8_t)M2.get_temperature();
    celM3 = (uint8_t)M3.get_temperature();
    celM4 = (uint8_t)M4.get_temperature();
    celM5 = (uint8_t)M5.get_temperature();
    celM6 = (uint8_t)M6.get_temperature();

    pros::screen::print(pros::E_TEXT_SMALL, 0, "%05d", pros::millis());
    pros::screen::print(pros::E_TEXT_SMALL, 1, "%s", eMsg.content.c_str());
    pros::screen::print(pros::E_TEXT_SMALL, 2, "%02d", celMI);
    pros::screen::print(pros::E_TEXT_SMALL, 3, "%02d", celM1);
    pros::screen::print(pros::E_TEXT_SMALL, 4, "%02d", celM2);
    pros::screen::print(pros::E_TEXT_SMALL, 5, "%02d", celM3);
    pros::screen::print(pros::E_TEXT_SMALL, 6, "%02d", celM4);
    pros::screen::print(pros::E_TEXT_SMALL, 7, "%02d", celM5);
    pros::screen::print(pros::E_TEXT_SMALL, 8, "%02d", celM6);
  }
}