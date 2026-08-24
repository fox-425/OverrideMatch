#include "autons/move.h"

extern Drive chassis;
extern pros::Imu inert;
extern Tracker tracker;

namespace Move {
  namespace MU {
    void driveChassis(float left_power, float right_power) {
      chassis.millivolts(left_power*1000, right_power*1000);
    }

    float normalizeTarget(float angle) {
      float current_heading = getInertialHeading();
      if (angle - current_heading > 180) {
        while (angle - current_heading > 180) angle -= 360;
      } else if (angle - current_heading < -180) {
        while (angle - current_heading < -180) angle += 360;
      }
      return angle;
    }

    float getInertialHeading() {
      return static_cast<float>(inert.get_rotation());
    }

    float getLeftRotationDegree() {
      return chassis.getPosition(-1);
    }

    float getRightRotationDegree() {
      return chassis.getPosition(1);
    }

    double getX() {
      return tracker.getX();
    }

    double getY() {
      return tracker.getY();
    }

    void scaleToMin(float& left_output, float& right_output, float min_output) {
      if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
        right_output = right_output / left_output * min_output;
        left_output = min_output;
      } else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
        left_output = left_output / right_output * min_output;
        right_output = min_output;
      } else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
        right_output = right_output / left_output * -min_output;
        left_output = -min_output;
      } else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
        left_output = left_output / right_output * -min_output;
        right_output = -min_output;
      }
    }

    void scaleToMax(float& left_output, float& right_output, float max_output) {
      if (fabs(left_output) >= fabs(right_output) && left_output > max_output) {
        right_output = right_output / left_output * max_output;
        left_output = max_output;
      } else if (fabs(right_output) > fabs(left_output) && right_output > max_output) {
        left_output = left_output / right_output * max_output;
        right_output = max_output;
      } else if (fabs(left_output) > fabs(right_output) && left_output < -max_output) {
        right_output = right_output / left_output * -max_output;
        left_output = -max_output;
      } else if (fabs(right_output) > fabs(left_output) && right_output < -max_output) {
        left_output = left_output / right_output * -max_output;
        right_output = -max_output;
      }
    }

    float absClamp(float value, float min, float max) {
      int32_t sign = utils::sign(value);
      float absV = fabs(value);
      if (absV < min) { absV = min; } else if (absV > max) { absV = max; }
      return absV*sign;
    }

    void stopChassis(motor_brake_mode_e_t type) {
      chassis.brake(type);
    }

    void resetChassis() {
      chassis.tare_position_all();
    }
  }

  //================================================================================================================================================================================================================================================================================
  //================================================================================================================================================================================================================================================================================
  //================================================================================================================================================================================================================================================================================

  void correctHeading(double kp, double ki, double kd) {
    PID::PIDoutput output;
    PID pid = PID(correct_angle, kp, ki, kd);

    pid.setIntegral(0.0f, fabs(correct_angle) / 2.5);
    pid.setExit(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    pid.setArrive(false);

    while(true) {
      pid.setTarget(correct_angle);
      if(!is_turning) {
        output = pid.update(pros::millis(), MU::getInertialHeading());
        MU::driveChassis(output.out, -output.out);
      }
      pros::delay(10);
    }
  }

  void turnToAngle(
    float turn_angle, uint32_t time_limit_msec, bool overshoot,
    float kp, float ki, float kd,
    float dRange, float errorRange1, float errorRange2, uint32_t errorTime1, uint32_t errorTime2,
    float min_output, float max_output
  ) {
    is_turning = true;
    PID pid = PID(turn_angle, kp, ki, kd);

    // Normalize and set PID target
    turn_angle = MU::normalizeTarget(turn_angle);
    pid.setTarget(turn_angle);
    pid.setIntegral(0.0f, 3.0f);

    // PID loop for turning
    PID::PIDoutput output;
    uint32_t start_time = pros::millis();
    float current_heading = MU::getInertialHeading();

    int32_t dir = utils::sign(MU::normalizeTarget(turn_angle - correct_angle));
    while (!pid.exit() && pros::millis() - start_time <= time_limit_msec) {
      current_heading = MU::getInertialHeading();
      if (overshoot && utils::sign(MU::normalizeTarget(turn_angle - current_heading)) != dir) {
        break;
      }
      output = pid.update(pros::millis(), current_heading);

      output.out = MU::absClamp(output.out, min_output, max_output);
      MU::driveChassis(output.out, -output.out);

      pros::delay(10);
    }

    if(!overshoot) {
      MU::stopChassis(E_MOTOR_BRAKE_HOLD);
    }
    correct_angle = turn_angle;
    is_turning = false;
  }

  void turnToPoint(
    double x, double y, bool rev, uint32_t time_limit_msec, bool overshoot,
    float kp, float ki, float kd,
    float dRange, float errorRange1, float errorRange2, uint32_t errorTime1, uint32_t errorTime2,
    float min_output, float max_output
  ) {
    is_turning = true;                  // Set turning state

    float add = (rev) ? 180.0f : 0.0f; // Add 180 degrees if turning to face backward
    float turn_angle = MU::normalizeTarget(utils::radToDeg(atan2(x - MU::getX(), y - MU::getY())) + add);
    PID pid = PID(turn_angle, kp, ki, kd);
    pid.setIntegral(0.0f, 3.0f);

    // PID loop for turning
    PID::PIDoutput output;
    uint32_t start_time = pros::millis();
    float current_heading = MU::getInertialHeading();

    int32_t dir = utils::sign(MU::normalizeTarget(turn_angle - correct_angle));
    while (!pid.exit() && pros::millis() - start_time <= time_limit_msec) {
      current_heading = MU::getInertialHeading();
      turn_angle = MU::normalizeTarget(utils::radToDeg(atan2(x - MU::getX(), y - MU::getY())) + add);
      pid.setTarget(turn_angle);
      if (overshoot && utils::sign(MU::normalizeTarget(turn_angle - current_heading)) != dir) {
        break;
      }
      output = pid.update(pros::millis(), current_heading);

      output.out = MU::absClamp(output.out, min_output, max_output);
      MU::driveChassis(output.out, -output.out);

      pros::delay(10);
    }

    if(!overshoot) {
      MU::stopChassis(E_MOTOR_BRAKE_HOLD);
    }
    correct_angle = turn_angle;
    is_turning = false;
  }
}