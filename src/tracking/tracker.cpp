#include "tracking/tracker.h"
#include "utils.h"

Tracker::TimeStamp::TimeStamp(uint32_t millis, double rot, int32_t centiVert, int32_t centiHori):
  millis(millis), rot(rot), centiVert(centiVert), centiHori(centiHori)
{}

Tracker::Tracker(Drive& chassis, Odom& odom, Imu& imu, Rotation& rotVert, Rotation& rotHori):
  chassis(chassis), odom(odom), imu(imu), rotVert(rotVert), rotHori(rotHori), past(0, 0, 0, 0),
  x(0), y(0)
{}

void Tracker::updateInit() {
  pros::Task tracking([this]() {
    this->updateLoop();
  });
}

void Tracker::updateLoop() {
  while (true) {
    update();
    pros::delay(10);
  }
}

void Tracker::update() {
  uint32_t millis = pros::millis();
  double rads = utils::degToRad(imu.get_rotation());
  int32_t centiVert = rotVert.get_position();
  int32_t centiHori = rotHori.get_position();

  odom.update(millis, rads, centiVert, centiHori);

  x += odom.dx;
  y += odom.dy;
}

void Tracker::reset() {
  imu.set_rotation(0);
  rotVert.set_position(0);
  rotHori.set_position(0);
}

void Tracker::setPos(double x, double y, double h) {
  rotVert.set_position(0);
  rotHori.set_position(0);
  odom.lastCentiHori = 0;
  odom.lastCentiVert = 0;
  odom.dx = 0;
  odom.dy = 0;
  imu.set_heading(h);
  this->x = x;
  this->y = y;
}

double Tracker::getX() {return x;}
double Tracker::getY() {return y;}
double Tracker::getH() {return imu.get_heading();}