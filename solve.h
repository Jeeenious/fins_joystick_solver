/*******************************************************************************
 * solve.h — 纯 C++ 速度解算，替代 Python solver_py
 ******************************************************************************/
#pragma once

#include <algorithm>

namespace solve {

inline constexpr float DIRECTION_THRESHOLD = 0.1f;
inline constexpr float DEPTH_THRESHOLD    = 0.2f;
inline constexpr float ROTATE_THRESHOLD   = 0.2f;
inline constexpr int   MAX_SPEED          = 99;
inline constexpr int   PWM_MIN            = 500;
inline constexpr int   PWM_MAX            = 2500;
inline constexpr int   SERVO_STEP         = 20;

inline int calc_speed(float v, float threshold) {
  if (-threshold < v && v < threshold) return 0;
  if (v > 0) return static_cast<int>((v - threshold) * MAX_SPEED / (1.0f - threshold));
  return -static_cast<int>((-v - threshold) * MAX_SPEED / (1.0f - threshold));
}

inline void solve_motion(float lh, float lv, float rh, float rv,
                         int &x, int &y, int &yaw, int &depth) {
  x     = calc_speed(lh, DIRECTION_THRESHOLD);
  y     = -calc_speed(lv, DIRECTION_THRESHOLD);
  yaw   = (-ROTATE_THRESHOLD < rh && rh < ROTATE_THRESHOLD) ? 0 : calc_speed(rh, ROTATE_THRESHOLD);
  depth = (-DEPTH_THRESHOLD < rv && rv < DEPTH_THRESHOLD) ? 0 : calc_speed(rv, DEPTH_THRESHOLD);
}

inline bool solve_servo_x(int hat_y, int &pwm) {
  if (hat_y == 1)       { pwm = std::min(pwm + SERVO_STEP, PWM_MAX); return true; }
  else if (hat_y == -1) { pwm = std::max(pwm - SERVO_STEP, PWM_MIN); return true; }
  return false;
}

inline bool solve_servo_y(int hat_x, int &pwm) {
  if (hat_x == 1)       { pwm = std::min(pwm + SERVO_STEP, PWM_MAX); return true; }
  else if (hat_x == -1) { pwm = std::max(pwm - SERVO_STEP, PWM_MIN); return true; }
  return false;
}

inline int solve_gripper(bool close, bool open) {
  if (close && !open) return 1;
  if (open && !close) return -1;
  return 0;
}

}  // namespace solve
