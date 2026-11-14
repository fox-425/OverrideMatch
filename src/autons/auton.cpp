#include "autons/auton.h"
extern Drive chassis;
extern Tracker tracker;

namespace auton {
  void test() {
    // Move::turnToAngle(90, 5000, false, 0.35, 0.15, 2.0, 4.5, 1.0, 3.0, 100, 300);
    // Move::moveToPose(10, 10, 0, 5000, false, true, 1.75, 0.0, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300);
    Move::MU::setLevel(0);
  }

  void sec15L() {
    // Move::MU::driveChassis(-1.0, 0, 0);
    Move::MU::toColor(1, 2000);
    Move::MU::setLevel(1);
    Move::moveToPose(17, 13, 60, 1500, false, true, 1.75, 0.0, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300);
    pros::delay(150);
    chassis.clawT();
    pros::delay(100);
    Move::MU::driveChassis(4, -6, 3);
    while (Move::MU::getInertialHeading() < 90+45-7.5) {
      pros::delay(10);
    }
    tracker.setPos(0,0,Move::MU::getInertialHeading());
    Move::moveToPose(-6, 6, 90+45, 5000, false, true, 1.75, 0.05, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300);
    Move::MU::setLevel(2);
    Move::turnToAngle(45, 5000, false, 0.35, 0.15, 2.0, 4.5, 1.0, 3.0, 100, 300);
    Move::moveToPose(-4, 9, 45, 2000, true, true, 1.0, 0.025, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300, 0, 6);
    Move::moveToPose(-1, 11.5, 45, 2000, true, true, 1.0, 0.025, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300, 0, 1.0, 0.15, 12);
    chassis.clawT();
    Move::MU::setLevel(3);
    pros::delay(150);
    Move::moveToPose(8, -1, 180, 2000, false, true, 1.75, 0.0, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300, 0, 3);
    pros::delay(150);
    chassis.clawT();
    // Move::MU::setLevel(0);
  }

  void sec15R() {
    chassis.millivolts(-6000, -6000, -6000, -6000);
    pros::delay(50);
    chassis.millivolts(0, 0, 0, 0);
    Move::MU::toColor(1, 2000);
    Move::MU::setLevel(1);
    Move::moveToPose(-17, 13, -60, 1500, false, true, 1.75, 0.0, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300);
    pros::delay(150);
    chassis.clawT();
    pros::delay(100);
    Move::MU::driveChassis(4, 6, -3);
    while (Move::MU::getInertialHeading() > -90-45+7.5) {
      pros::delay(10);
    }
    tracker.setPos(0,0,Move::MU::getInertialHeading());
    Move::moveToPose(6, 6, -90-45, 5000, false, true, 1.75, 0.05, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300);
    Move::MU::setLevel(2);
    Move::turnToAngle(-45, 3000, false, 0.35, 0.15, 2.0, 4.5, 1.0, 3.0, 100, 300);
    Move::moveToPose(1.5, 9.5, -45, 2000, true, true, 1.0, 0.025, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300, 0, 6);
    Move::moveToPose(-1.5, 11.5, -45, 2000, true, true, 1.0, 0.025, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300, 0, 1.0, 0.15, 12);
    chassis.clawT();
    Move::MU::setLevel(3);
    pros::delay(150);
    Move::moveToPose(-7.5, 1, 180, 1500, false, true, 1.75, 0.0, 12.0, 0.35, 0.15, 2.0, 4.5, 0.15, 0.4, 100, 300, 4.5, 1, 3, 100, 300, 0, 3);
    pros::delay(150);
    chassis.clawT();
  }
}