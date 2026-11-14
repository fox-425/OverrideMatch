#include "data/screenWriting.h"
#include "data/EMSG.h"
#include "data/dataLogger.h"
#include "pros/screen.hpp"
#include "pros/motors.hpp"
#include "autons/move.h"

extern EMSG eMsg;
extern Logger logger;
extern pros::Motor M1;
extern pros::Motor M2;
extern pros::Motor M3;
extern pros::Motor M4;
extern pros::Motor MC1;
extern pros::Motor MC2;
extern pros::Motor MC3;
extern pros::Rotation RY;
extern pros::Rotation RX;

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
      logger.addToBuffer(Data(
        pros::millis(),
        RY.get_position()/100.0, RX.get_position()/100.0, 0, 0, 0, 0, 0,
        Move::MU::getInertialHeading(), Move::MU::getX(), Move::MU::getY()
      ));
      pros::delay(100);
    }
  }

  void update() {
    x = Move::MU::getX();
    y = Move::MU::getY();
    h = Move::MU::getInertialHeading();
    celM1 = (uint8_t)M1.get_temperature();
    celM2 = (uint8_t)M2.get_temperature();
    celM3 = (uint8_t)M3.get_temperature();
    celM4 = (uint8_t)M4.get_temperature();
    mvM1 = M1.get_voltage();
    mvM2 = M2.get_voltage();
    mvM3 = M3.get_voltage();
    mvM4 = M4.get_voltage();
    celMC1 = (uint8_t)MC1.get_temperature();
    celMC2 = (uint8_t)MC2.get_temperature();
    celMC3 = (uint8_t)MC3.get_temperature();
    mvMC1 = MC1.get_voltage();
    mvMC2 = MC2.get_voltage();
    mvMC3 = MC3.get_voltage();
    amMC1 = MC1.get_current_draw();
    amMC2 = MC2.get_current_draw();
    amMC3 = MC3.get_current_draw();

    pros::screen::print(pros::E_TEXT_SMALL, 0, "%05d", pros::millis());
    pros::screen::print(pros::E_TEXT_SMALL, 1, "%s", eMsg.content.c_str());
    pros::screen::print(pros::E_TEXT_SMALL, 2, "%05.2f %05.2f %05.2f", x, y, h);
    pros::screen::print(pros::E_TEXT_SMALL, 3, "%02d %06d", celM1, mvM1);
    pros::screen::print(pros::E_TEXT_SMALL, 4, "%02d %06d", celM2, mvM2);
    pros::screen::print(pros::E_TEXT_SMALL, 5, "%02d %06d", celM3, mvM3);
    pros::screen::print(pros::E_TEXT_SMALL, 6, "%02d %06d", celM4, mvM4);
    pros::screen::print(pros::E_TEXT_SMALL, 7, "%02d %06d %05d", celMC1, mvMC1, amMC1);
    pros::screen::print(pros::E_TEXT_SMALL, 8, "%02d %06d %05d", celMC2, mvMC2, amMC2);
    pros::screen::print(pros::E_TEXT_SMALL, 9, "%02d %06d %05d", celMC3, mvMC3, amMC3);

    // pros::screen::draw_pixel(479, 0);
    // pros::screen::draw_pixel(479, 239);
  }
}
