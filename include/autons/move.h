#pragma once
#include "drive/drive.h"
#include "tracking/tracker.h"
#include "pid.h"
#include <cmath>

namespace Move {
  namespace MU {
    void driveChassis(float left_power, float right_power);
    float normalizeTarget(float angle);
    float getInertialHeading();
    float getLeftRotationDegree();
    float getRightRotationDegree();
    double getX();
    double getY();
    void scaleToMin(float& left_output, float& right_output, float min_output);
    void scaleToMax(float& left_output, float& right_output, float max_output);
    float absClamp(float value, float min, float max);
    void stopChassis(motor_brake_mode_e_t type);
    void resetChassis();
  }

  /** @brief When true, the robot will attempt to turn to correct_angle */
  inline bool is_turning = false;

  /** @brief The angle the robot will attempt to turn to when is_turning is false. The value should be the last angle the robots was told to turn to */
  inline float correct_angle = 0.0f;


  //================================================================================================================================================================================================================================================================================
  //================================================================================================================================================================================================================================================================================
  //================================================================================================================================================================================================================================================================================

  /**
   * @brief Corrects the robot's heading to the correct_angle using a PID controller
  */
  void correctHeading(double kp, double ki, double kd);

  /**
   * @brief Turns the robot to a specified angle using PID control.
   * @param turn_angle: Target angle to turn to (in degrees).
   * @param time_limit_msec: Maximum time allowed for the turn (in milliseconds).
   * @param overshoot: If true, overshooting the target exits; if false, follows normal behavior.
   * @param max_output: Maximum voltage output to motors.
   */
  void turnToAngle(
    float turn_angle, uint32_t time_limit_msec, bool overshoot,
    float kp, float ki, float kd,
    float dRange = 4.5f, float errorRange1 = 1.0f, float errorRange2 = 3.0f, uint32_t errorTime1 = 50, uint32_t errorTime2 = 250,
    float min_output = 0.0f, float max_output = 12.0f
  );

  /**
   * @brief Turns the robot to face a specific point in the field.
   * @param x, y: Coordinates of the target point.
   * @param direction: Direction to face the point (1 for forward, -1 for backward).
   * @param time_limit_msec: Maximum time allowed for the turn (in milliseconds).
   */
  void turnToPoint(
    double x, double y, bool rev, uint32_t time_limit_msec, bool overshoot,
    float kp, float ki, float kd,
    float dRange = 4.5f, float errorRange1 = 1.0f, float errorRange2 = 3.0f, uint32_t errorTime1 = 50, uint32_t errorTime2 = 250,
    float min_output = 0.0f, float max_output = 12.0f
  );

  void moveToPoint(
    double x, double y, int32_t dir, uint32_t time_limit_msec, bool chain,
    double d_kp, double d_ki, double d_kd, double h_kp, double h_ki, double h_kd,
    bool dir_change_start, bool dir_change_end,
    double min_output = 0, double max_output = 12,
    double max_slew_accel_fwd = 12, double max_slew_decel_rev = 12, double max_slew_decel_fwd = 12, double max_slew_accel_rev = 12
  );

}