#include "autons/move.h"

extern Drive chassis;
extern Tracker tracker;

namespace Move {
  namespace MU {
    void driveChassis(float fwrd, float side, float turn, float accel, float decel, MVs* pMVs) {
      constexpr double mult = 1000.0;
      fwrd *= mult;
      side *= mult;
      turn *= mult;
      double mv1 = fwrd + side + turn;
      double mv2 = fwrd - side - turn;
      double mv3 = fwrd + side - turn;
      double mv4 = fwrd - side + turn;
      double max = std::max({std::abs(mv1), std::abs(mv2), std::abs(mv3), std::abs(mv4)});
      if (max > 12000) {
        double mult = 12000.0 / max;
        mv1 *= mult;
        mv2 *= mult;
        mv3 *= mult;
        mv4 *= mult;
      }
      if ((accel > 0 || decel > 0) && pMVs != nullptr) {
        if(pMVs->MV1 - mv1 > decel) {mv1 = pMVs->MV1 - decel;}
        if(pMVs->MV2 - mv2 > decel) {mv2 = pMVs->MV2 - decel;}
        if(pMVs->MV3 - mv3 > decel) {mv3 = pMVs->MV3 - decel;}
        if(pMVs->MV4 - mv4 > decel) {mv4 = pMVs->MV4 - decel;}
        if(mv1 - pMVs->MV1 > accel) {mv1 = pMVs->MV1 + accel;}
        if(mv2 - pMVs->MV2 > accel) {mv2 = pMVs->MV2 + accel;}
        if(mv3 - pMVs->MV3 > accel) {mv3 = pMVs->MV3 + accel;}
        if(mv4 - pMVs->MV4 > accel) {mv4 = pMVs->MV4 + accel;}
      }
      chassis.millivolts(mv1, mv2, mv3, mv4);
      if (pMVs != nullptr) {
        pMVs->MV1 = mv1;
        pMVs->MV2 = mv2;
        pMVs->MV3 = mv3;
        pMVs->MV4 = mv4;
      }
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
      return static_cast<float>(tracker.imu.get_rotation());
    }
    double getX() {return tracker.getX();}
    double getY() {return tracker.getY();}

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
        MU::driveChassis(0, 0, output.out);
      }
      pros::delay(10);
    }
  }

  void turnToAngle(
    float turn_angle, uint32_t time_limit_msec, bool overshoot,
    float kp, float ki, float kd,
    float dRange, float errorRange1, float errorRange2, uint32_t errorTime1, uint32_t errorTime2,
    bool stop, float min_output, float max_output
  ) {
    is_turning = true;
    PID pid = PID(turn_angle, kp, ki, kd);

    // Normalize and set PID target
    turn_angle = MU::normalizeTarget(turn_angle);
    pid.setTarget(turn_angle);
    pid.setExit(dRange, errorRange1, errorRange2, errorTime1, errorTime2);
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
      MU::driveChassis(0, 0, output.out);

      pros::delay(10);
    }

    if(stop) {
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
      MU::driveChassis(0, 0, output.out);

      pros::delay(10);
    }

    if(!overshoot) {
      MU::stopChassis(E_MOTOR_BRAKE_HOLD);
    }
    correct_angle = turn_angle;
    is_turning = false;
  }

  void moveToPoint(
    double x, double y, bool rev, uint32_t time_limit_msec, bool chain, bool stop,
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

    PID pid_d = PID(hypot(x - MU::getX(), y - MU::getY()), d_kp, d_ki, d_kd);
    PID pid_h = PID(MU::normalizeTarget(utils::radToDeg(atan2(x - MU::getX(), y - MU::getY())) + add), h_kp, h_ki, h_kd);

    // Set PID targets for distance and heading
    pid_d.setIntegral(0.0f, 3.3f);
    pid_d.setExit(5.0f, 0.5f, 1.5f, 50, 250);

    pid_h.setIntegral(0.0f, 1.0f);
    pid_h.setExit(0.0f, 0.0f, 0.0f, 0, 0);
    pid_h.setArrive(false);

    // Reset the chassis
    uint32_t start_time = pros::millis();
    float left_output = 0.0f, right_output = 0.0f, prev_left_output = 0.0f, prev_right_output = 0.0f;
    PID::PIDoutput correction_output;
    bool perpendicular_line = false, prev_perpendicular_line = true;

    double current_angle = 0, overturn_value = 0;
    bool ch = true;
    int32_t dir = (rev) ? -1 : 1;

    // Main PID loop for moving to point
    while (pros::millis() - start_time <= time_limit_msec) {
      // Continuously update targets as robot moves
      pid_h.setTarget(MU::normalizeTarget(utils::radToDeg(atan2(x - MU::getX(), y - MU::getY())) + add));
      pid_d.setTarget(hypot(x - MU::getX(), y - MU::getY()));
      current_angle = MU::getInertialHeading();
      // Calculate drive output based on heading and distance
      left_output = pid_d.update(pros::millis(), 0).out * cos(utils::degToRad(atan2(x - MU::getX(), y - MU::getY()) * 180 / M_PI + add - current_angle)) * dir;
      right_output = left_output;
      // Check if robot has crossed the perpendicular line to the target
      perpendicular_line = ((MU::getY() - y) * -cos(utils::degToRad(MU::normalizeTarget(current_angle + add))) <= (MU::getX() - x) * sin(utils::degToRad(MU::normalizeTarget(current_angle + add))) + overtol);
      if(perpendicular_line && !prev_perpendicular_line) {
        break;
      }
      prev_perpendicular_line = perpendicular_line;

      // Only apply heading correction if far from target
      if(hypot(x - MU::getX(), y - MU::getY()) > 8 && ch == true) {
        correction_output = pid_h.update(pros::millis(), current_angle);
      } else {
        correction_output.out = 0;
        ch = false;
      }

      // Minimum Output Check
      if(min_speed) {
        MU::scaleToMin(left_output, right_output, min_output);
      }

      // Overturn logic for sharp turns
      overturn_value = fabs(left_output) + fabs(correction_output.out) - max_output;
      if(overturn_value > 0) {
        left_output += (left_output > 0) ? -overturn_value : overturn_value;
      }
      right_output = left_output;
      left_output = left_output + correction_output.out;
      right_output = right_output - correction_output.out;

      // Max Output Check
      MU::scaleToMax(left_output, right_output, max_output);

      // Max Acceleration/Deceleration Check
      if(prev_left_output - left_output > slew_rev) {
        left_output = prev_left_output - slew_rev;
      }
      if(prev_right_output - right_output > slew_rev) {
        right_output = prev_right_output - slew_rev;
      }
      if(left_output - prev_left_output > slew_fwd) {
        left_output = prev_left_output + slew_fwd;
      }
      if(right_output - prev_right_output > slew_fwd) {
        right_output = prev_right_output + slew_fwd;
      }
      prev_left_output = left_output;
      prev_right_output = right_output;
      // MU::driveChassis(left_output, right_output, kidasnas); // Apply output to chassis
      pros::delay(10);
    }
    correct_angle = MU::getInertialHeading(); // Update global heading
    is_turning = false;                   // Reset turning state
  }

  void moveToPose(
    double x, double y, double angle, uint32_t time_limit_msec, bool chain, bool stop,
    float d_kp, float d_ki, float d_kd, float h_kp, float h_ki, float h_kd,
    float d_DR, float d_ER1, float d_ER2, uint32_t d_ET1, uint32_t d_ET2,
    float h_DR, float h_ER1, float h_ER2, uint32_t h_ET1, uint32_t h_ET2,
    float overtol,
    float min_output, float max_output,
    float max_accel, float max_decel
  ) {
    is_turning = true;

    PID pid_x(x - MU::getX(), d_kp, d_ki, d_kd);
    PID pid_y(y - MU::getY(), d_kp, d_ki, d_kd);
    PID pid_h(MU::normalizeTarget(angle), h_kp, h_ki, h_kd);

    pid_x.setIntegral(0.0, 3.3);
    pid_y.setIntegral(0.0, 3.3);
    pid_h.setIntegral(0.0, 1.0);
    pid_x.setExit(d_DR, d_ER1, d_ER2, d_ET1, d_ET2);
    pid_y.setExit(d_DR, d_ER1, d_ER2, d_ET1, d_ET2);
    pid_h.setExit(h_DR, h_ER1, h_ER2, h_ET1, h_ET2);
    pid_x.setArrive(false);
    pid_y.setArrive(false);
    pid_h.setArrive(false);

    uint32_t start = pros::millis();
    MVs pMVs{};
    while (pros::millis() - start < time_limit_msec) {
      double ex = x - MU::getX();
      double ey = y - MU::getY();
      double dist = hypot(ex, ey);

      pid_x.setTarget(x);
      pid_y.setTarget(y);

      float x_out = pid_x.update(pros::millis(), MU::getX()).out;
      float y_out = pid_y.update(pros::millis(), MU::getY()).out;

      double head = MU::getInertialHeading();
      double theta = utils::degToRad(head);
      
      float fwrd = x_out * sin(theta) + y_out * cos(theta);
      float side = x_out * cos(theta) - y_out * sin(theta);
      
      pid_h.setTarget(MU::normalizeTarget(angle));

      float turn = pid_h.update(pros::millis(), head).out;

      float mag = hypot(fwrd, side);
      if (dist > overtol && mag > 0 && mag < min_output) {
        float scale = min_output / mag;
        fwrd *= scale;
        side *= scale;
      }
      float mv1 = fwrd + side + turn;
      float mv2 = fwrd - side - turn;
      float mv3 = fwrd + side - turn;
      float mv4 = fwrd - side + turn;

      float max_mv = std::max({std::fabs(mv1), std::fabs(mv2), std::fabs(mv3), std::fabs(mv4)});
      if (max_mv > max_output) {
        float scale = max_output / max_mv;
        fwrd *= scale;
        side *= scale;
        turn *= scale;
      }
      MU::driveChassis(fwrd, side, turn, max_accel, max_decel, &pMVs);

      if (pid_x.exit() && pid_y.exit() && pid_h.exit()) {
        break;
      }

      pros::delay(10);
    }
    correct_angle = angle;
    is_turning = false;
  }
}