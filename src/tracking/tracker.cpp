#include "tracking/tracker.h"
#include "utils.h"

Tracker::TimeStamp::TimeStamp(uint32_t millis, double rot, int32_t centiVert, int32_t centiHori):
  millis(millis), rot(rot), centiVert(centiVert), centiHori(centiHori)
{}

Tracker::Tracker(Drive& chassis, Odom& odom, Imu& imu, Rotation& rotVert, Rotation& rotHori):
  chassis(chassis), odom(odom), imu(imu), rotVert(rotVert), rotHori(rotHori), past(0, 0, 0, 0),
  x(0), y(0)
{}

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