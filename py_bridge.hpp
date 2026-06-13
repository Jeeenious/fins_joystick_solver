/*******************************************************************************
 * py_bridge.hpp — 速度解算接口（纯数学，不含协议编码）
 ******************************************************************************/
#pragma once

#include <string>

namespace py_bridge {

void init();

std::string solve_motion(float lh, float lv, float rh, float rv);
bool        solve_servo(int hat_x, int hat_y, int &pwm_x, int &pwm_y);
int         solve_gripper(bool close_btn, bool open_btn);

}  // namespace py_bridge
