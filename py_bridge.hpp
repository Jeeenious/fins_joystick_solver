/*******************************************************************************
 * py_bridge.hpp — 速度解算接口（纯数学，不含协议编码）
 ******************************************************************************/
#pragma once

namespace py_bridge {

void init();

// 通过引用返回四个推力值
void solve_motion(float lh, float lv, float rh, float rv,
                  int &x, int &y, int &yaw, int &depth);

// 各轴独立解算，返回是否有变化
bool solve_servo_x(int hat_y, int &pwm);
bool solve_servo_y(int hat_x, int &pwm);

// 返回夹爪方向 -1/0/1
int solve_gripper(bool close_btn, bool open_btn);

}  // namespace py_bridge
