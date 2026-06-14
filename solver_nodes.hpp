/*******************************************************************************
 * solver_nodes.hpp — 解算 FINS 节点（纯数学，不含协议编码）
 *
 * 全部输入/输出均为标量端口，零 array / pair / vector:
 *   lh,lv,rh,rv (float) → MotionSolver          → x,y,yaw,depth (int)
 *   hat_y (int)         → ServoXSolver (SISO)   → pwm_x (int)
 *   hat_x (int)         → ServoYSolver (SISO)   → pwm_y (int)
 *   btn_4,btn_5 (bool)  → GripperSolver         → direction (int)
 ******************************************************************************/

#pragma once

#include <fins/node.hpp>
#include "py_bridge.hpp"

// ──────────── MotionSolver ────────────
class MotionSolver : public fins::Node {
public:
  MotionSolver() = default;

  void define() override {
    set_name("MotionSolver");
    set_description("摇杆轴值 → 运动推力百分比");
    set_category("JoystickSolver");
    register_input<float>("lh", &MotionSolver::on_lh);
    register_input<float>("lv", &MotionSolver::on_lv);
    register_input<float>("rh", &MotionSolver::on_rh);
    register_input<float>("rv", &MotionSolver::on_rv);
    register_output<int>("x");
    register_output<int>("y");
    register_output<int>("yaw");
    register_output<int>("depth");
  }
  void initialize() override { logger->info("MotionSolver 初始化."); lh_=lv_=rh_=rv_=0; }
  void run() override       { logger->info("MotionSolver 运行."); }
  void pause() override     { logger->info("MotionSolver 暂停."); }
  void reset() override     { logger->info("MotionSolver 重置."); }

  void on_lh(const float &v, fins::AcqTime) { lh_=v; try_send(); }
  void on_lv(const float &v, fins::AcqTime) { lv_=v; try_send(); }
  void on_rh(const float &v, fins::AcqTime) { rh_=v; try_send(); }
  void on_rv(const float &v, fins::AcqTime) { rv_=v; try_send(); }
private:
  float lh_, lv_, rh_, rv_;
  void try_send() {
    int x, y, yaw, depth;
    py_bridge::solve_motion(lh_, lv_, rh_, rv_, x, y, yaw, depth);
    send("x", x);
    send("y", y);
    send("yaw", yaw);
    send("depth", depth);
  }
};
EXPORT_NODE(MotionSolver)

// ──────────── ServoXSolver (SISO: hat_y → pwm_x) ────────────
class ServoXSolver : public fins::Node {
public:
  ServoXSolver() = default;

  void define() override {
    set_name("ServoXSolver");
    set_description("十字键上下 → 舵机 X PWM 增量");
    set_category("JoystickSolver");
    register_input<int>("hat_y", &ServoXSolver::on_hat_y);
    register_output<int>("pwm_x");
  }
  void initialize() override { logger->info("ServoXSolver 初始化."); pwm_ = 1500; }
  void run() override       { logger->info("ServoXSolver 运行."); }
  void pause() override     { logger->info("ServoXSolver 暂停."); }
  void reset() override     { logger->info("ServoXSolver 重置."); }

  void on_hat_y(const int &y, fins::AcqTime) {
    if (py_bridge::solve_servo_x(y, pwm_))
      send("pwm_x", pwm_);
  }
private:
  int pwm_;
};
EXPORT_NODE(ServoXSolver)

// ──────────── ServoYSolver (SISO: hat_x → pwm_y) ────────────
class ServoYSolver : public fins::Node {
public:
  ServoYSolver() = default;

  void define() override {
    set_name("ServoYSolver");
    set_description("十字键左右 → 舵机 Y PWM 增量");
    set_category("JoystickSolver");
    register_input<int>("hat_x", &ServoYSolver::on_hat_x);
    register_output<int>("pwm_y");
  }
  void initialize() override { logger->info("ServoYSolver 初始化."); pwm_ = 1500; }
  void run() override       { logger->info("ServoYSolver 运行."); }
  void pause() override     { logger->info("ServoYSolver 暂停."); }
  void reset() override     { logger->info("ServoYSolver 重置."); }

  void on_hat_x(const int &x, fins::AcqTime) {
    if (py_bridge::solve_servo_y(x, pwm_))
      send("pwm_y", pwm_);
  }
private:
  int pwm_;
};
EXPORT_NODE(ServoYSolver)

// ──────────── GripperSolver ────────────
class GripperSolver : public fins::Node {
public:
  GripperSolver() = default;

  void define() override {
    set_name("GripperSolver");
    set_description("按钮 → 夹爪方向解算");
    set_category("JoystickSolver");
    register_input<bool>("btn_4", &GripperSolver::on_btn_4);
    register_input<bool>("btn_5", &GripperSolver::on_btn_5);
    register_output<int>("direction");
  }
  void initialize() override { logger->info("GripperSolver 初始化."); close_ = open_ = false; }
  void run() override       { logger->info("GripperSolver 运行."); }
  void pause() override     { logger->info("GripperSolver 暂停."); }
  void reset() override     { logger->info("GripperSolver 重置."); }

  void on_btn_4(const bool &pressed, fins::AcqTime) { close_ = pressed; try_send(); }
  void on_btn_5(const bool &pressed, fins::AcqTime) { open_  = pressed; try_send(); }
private:
  bool close_, open_;
  void try_send() {
    send("direction", py_bridge::solve_gripper(close_, open_));
  }
};
EXPORT_NODE(GripperSolver)
