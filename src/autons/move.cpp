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

void moveToPoint(
  double x, double y, bool rev, uint32_t time_limit_msec, bool chain,
    float d_kp, float d_ki, float d_kd, float h_kp, float h_ki, float h_kd,
    float overtol, bool dir_change_start, bool dir_change_end,
    float min_output, float max_output,
    float max_accel, float max_decel
) {
  is_turning = true;                  // Set turning state
  float add = (rev) ? 180.0f : 0.0f;
  float slew_fwd = (rev) ? max_decel : max_accel;
  float slew_rev = (rev) ? max_accel : max_decel;
  bool min_speed = false;
  if(chain) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      slew_fwd = (rev) ? max_decel : 24.0f;
      slew_rev = (rev) ? 24.0f : max_decel;
    }
    if(dir_change_start && !dir_change_end) {
      slew_fwd = (rev) ? 24.0f : max_accel;
      slew_rev = (rev) ? max_accel : 24.0f;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      slew_fwd = 24.0f;
      slew_rev = 24.0f;
      min_speed = true;
    }
  }

  PID pid_d = PID(hypot(x - getX(), y - getY()), d_kp, d_ki, d_kd);
  PID pid_h = PID(normalizeTarget(radToDeg(atan2(x - getX(), y - getY())) + add), h_kp, h_ki, h_kd);

  // Set PID targets for distance and heading
  pid_d.setIntegral(0.0f, 3.3f);
  pid_d.setExit(5.0f, 0.5f, 1.5f, 50, 250);

  pid_h.setIntegral(0.0f, 1.0f);
  pid_h.setExit(0.0f, 0.0f, 0.0f, 0, 0);
  pid_h.setArrive(false);

  // Reset the chassis
  uint32_t start_time = millis();
  float left_output = 0.0f, right_output = 0.0f, correction_output = 0.0f, prev_left_output = 0.0f, prev_right_output = 0.0f;
  float exittolerance = 1.0f;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;
  int32_t dir = (rev) ? -1 : 1;

  // Main PID loop for moving to point
  while (pros::millis() - start_time <= time_limit_msec) {
    // Continuously update targets as robot moves
    pid_h.setTarget(normalizeTarget(radToDeg(atan2(x - getX(), y - getY())) + add));
    pid_d.setTarget(hypot(x - getX(), y - getY()));
    current_angle = getInertialHeading();
    // Calculate drive output based on heading and distance
    left_output = pid_d.update(0) * cos(degToRad(atan2(x - getX(), y - getY()) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((getY() - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (getX() - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Only apply heading correction if far from target
    if(hypot(x - getX(), y - getY()) > 8 && ch == true) {
      correction_output = pid_h.update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0) {
      left_output += (left_output > 0) ? -overturn_value : overturn_value;
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    pros::delay(10);
  }
  correct_angle = getInertialHeading(); // Update global heading
  is_turning = false;                   // Reset turning state
}
