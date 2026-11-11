#include "main.h"
#include "drive/drive.h"
#include "drive/control.h"
#include "tracking/tracker.h"
#include "tracking/odom.h"
#include "autons/move.h"
#include "autons/auton.h"
#include "data/EMSG.h"
#include "data/dataLogger.h"
#include "data/screenWriting.h"
#include "utils.h"
#include "pid.h"

#include <limits>
#include <cmath>
#include <arm_neon.h>

int8_t M1Port = -1;
int8_t M2Port = 10;
int8_t M3Port = 11;
int8_t M4Port = -20;
int8_t MC1Port = 2;
int8_t MC2Port = -3;
int8_t MC3Port = 4;
int8_t MTPort = -19;
char Cw_3Wire = 'G';
int8_t D1Port = 21;
int8_t D2Port = 21;
int8_t D3Port = 21;
int8_t D4Port = 21;
int8_t RYPort = 12;
int8_t RXPort = -5;
int8_t Iport = 7;
int8_t CrPort = 21;

EMSG eMsg;
Motor M1(M1Port, MotorGearset::blue);
Motor M2(M2Port, MotorGearset::blue);
Motor M3(M3Port, MotorGearset::blue);
Motor M4(M4Port, MotorGearset::blue);
Motor MC1(MC1Port, MotorGearset::blue);
Motor MC2(MC2Port, MotorGearset::blue);
Motor MC3(MC3Port, MotorGearset::blue);
Motor MT(MTPort, MotorGearset::blue);
pros::adi::DigitalOut claw(Cw_3Wire);
pros::Distance D1(D1Port);
pros::Distance D2(D2Port);
pros::Distance D3(D3Port);
pros::Distance D4(D4Port);
pros::Rotation RY(RYPort);
pros::Rotation RX(RXPort);
pros::Imu imu(Iport);
pros::Optical opt(CrPort);
pros::Controller primary(pros::E_CONTROLLER_MASTER);

Drive chassis(M1, M2, M3, M4, MC1, MC2, MC3, MT, claw, opt);
Odom odom(2, 2, 0.723852815581, 1.24661715039);
Tracker tracker(chassis, odom, imu, RY, RX);
Control control(chassis, primary);
Logger logger;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  M1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  M2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  M3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  M4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  MC1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  MC2.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  MC3.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  MT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  RY.reset_position();
  RX.reset_position();
  imu.reset(false);
  Printing::init();
  logger.startLogging();
  uint32_t time = pros::millis() + 2000;
  while(pros::millis() < time && imu.is_calibrating()) {
    pros::delay(10);
  }
  pros::delay(2000);
  tracker.setPos(0, 0);
  tracker.updateInit();
  pros::delay(50);
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
void autonomous() {
  auton::test();
}

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
  // while (!primary.get_digital(E_CONTROLLER_DIGITAL_A)) {
  //   pros::delay(10);
  // }
  // uint32_t start = pros::millis();
  // int32_t PY1 = RY.get_position();
  // int32_t PX1 = RX.get_position();
  // double rot1 = imu.get_rotation();
  // chassis.millivolts(6000, -6000, -6000, 6000);
  // while (imu.get_rotation()-rot1 < 90) {
  //   pros::delay(10);
  // }
  // int32_t PY2 = RY.get_position();
  // int32_t PX2 = RX.get_position();
  // double rot2 = imu.get_rotation();
  // uint32_t end = pros::millis();
  // chassis.millivolts(0, 0, 0, 0);
  // eMsg.clearText();
  // eMsg.addText("PY: " + std::to_string(PY2 - PY1) + ", PX: " + std::to_string(PX2 - PX1) + ", Rot: " + std::to_string(rot2 - rot1) + ", Time: " + std::to_string(end - start));
  // while (true) {
  //   pros::delay(10);
  // }

  /*
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
  */

  uint8_t prev = 0;
  MVs pMVs;
  PID pidH(0.0, 1.0, 0.0, 3.0, false);
  pidH.setExit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  pidH.setIntegral(0.0f, 0.0f);
  while (true) {
    control.Analog(pMVs, pidH);
    prev = control.Buttons(prev);

    pros::delay(10);
  }
  //*/
}
