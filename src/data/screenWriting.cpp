#include "data/screenWriting.h"
#include "pros/screen.hpp"
#include "pros/rtos.hpp"

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
    celM1 = (uint8_t)0.0;
    celM2 = (uint8_t)0.0;
    celM3 = (uint8_t)0.0;
    celM4 = (uint8_t)0.0;
    celM5 = (uint8_t)0.0;
    celM6 = (uint8_t)0.0;

    pros::screen::print(pros::E_TEXT_SMALL, 0, "%05d", pros::millis());
    pros::screen::print(pros::E_TEXT_SMALL, 1, "%02d", celM1);
    pros::screen::print(pros::E_TEXT_SMALL, 2, "%02d", celM2);
    pros::screen::print(pros::E_TEXT_SMALL, 3, "%02d", celM3);
    pros::screen::print(pros::E_TEXT_SMALL, 4, "%02d", celM4);
    pros::screen::print(pros::E_TEXT_SMALL, 5, "%02d", celM5);
    pros::screen::print(pros::E_TEXT_SMALL, 6, "%02d", celM6);
  }
}