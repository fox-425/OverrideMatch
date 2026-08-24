#include "main.h"
#include "drive/drive.h"
#include "drive/control.h"
#include "tracking/tracker.h"
#include "tracking/odom.h"
#include "autons/move.h"
#include "data/screenWriting.h"
#include "utils.h"
#include "pid.h"

#include <limits>
#include <cmath>
#include <arm_neon.h>

int8_t M1Port = 1;
int8_t M2Port = -2;
int8_t M3Port = 3;
int8_t M4Port = -4;
int8_t M5Port = 5;
int8_t M6Port = -6;
Motor M1(M1Port, MotorGearset::blue);
Motor M2(M2Port, MotorGearset::blue);
Motor M3(M3Port, MotorGearset::blue);
Motor M4(M4Port, MotorGearset::blue);
Motor M5(M5Port, MotorGearset::blue);
Motor M6(M6Port, MotorGearset::blue);
Drive chassis(M1, M2, M3, M4, M5, M6);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Printing::init();

	pros::delay(2000);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  
	PID testPID(100, 0.5, 0, 12);
  testPID.setExit(10, 2, 1, 500, 150);

  double val = 0.0;
  double velo = 0.0;

  std::vector<double> vals;

  uint32_t prevT = pros::millis();
  while (true) {
    PID::PIDoutput out = testPID.update(pros::millis(), val);
    velo += out.out*0.01;
    if (velo > 12) {
      velo = 12;
    } else if (velo < -12) {
      velo = -12;
    }
    val += velo;
    vals.push_back(val);

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(E_TEXT_MEDIUM, 0, "%04.2f", val);

    if (vals.size() > 200) {
      std::vector<double> temp;
      for (int i = 1; i < 201; i++) {
        temp.push_back(vals.at(i));
      }
      vals = temp;
    }
    int16_t x = 0;
    double origin = 200;
    double mult = 1;

    for (int i = 0; i < vals.size(); i++) {
      int16_t y = std::round(origin-vals.at(vals.size()-i-1)*mult);
      pros::screen::set_pen((fabs(out.d) <= 10) ? pros::Color::green : pros::Color::white);
      pros::screen::draw_pixel(x, y);

      pros::screen::set_pen(pros::Color::blue);
      y = std::round(origin-testPID.getTarget()*mult);
      pros::screen::draw_pixel(x, y);
      
      y = origin;
      pros::screen::draw_pixel(x,y);

      x += 2;
    }


    if (testPID.exit()) {
      break;
    }
    

    pros::Task::delay_until(&prevT, 100);
    pros::screen::erase();
  }

  pros::screen::print(E_TEXT_MEDIUM, 1, "%d", pros::millis());
  pros::screen::print(E_TEXT_MEDIUM, 2, "%d", testPID.exitType());
  // */
  while(true){
    pros::delay(5000);
  }
}