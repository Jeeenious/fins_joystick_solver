/*******************************************************************************
 * solver_nodes.hpp — 解算 FINS 节点（纯数学，不含协议编码）
 ******************************************************************************/

#pragma once

#include <sstream>
#include <vector>

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
    register_input<std::string>("axes", &MotionSolver::on_axes);
    register_output<std::string>("motion");
  }
  void initialize() override { logger->info("MotionSolver 初始化."); }
  void run() override       { logger->info("MotionSolver 运行."); }
  void pause() override     { logger->info("MotionSolver 暂停."); }
  void reset() override     { logger->info("MotionSolver 重置."); }

  void on_axes(const std::string &data, fins::AcqTime) {
    auto v = parse_floats(data);
    if (v.size() >= 4)
      send("motion", py_bridge::solve_motion(v[0], v[1], v[2], v[3]));
  }
private:
  static std::vector<float> parse_floats(const std::string &s) {
    std::vector<float> v;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
      v.push_back(std::stof(token));
    return v;
  }
};
EXPORT_NODE(MotionSolver)

// ──────────── ServoSolver ────────────
class ServoSolver : public fins::Node {
public:
  ServoSolver() = default;

  void define() override {
    set_name("ServoSolver");
    set_description("十字键 → 舵机 PWM 增量解算");
    set_category("JoystickSolver");
    register_input<std::string>("hat", &ServoSolver::on_hat);
    register_output<std::string>("motion");
  }
  void initialize() override {
    logger->info("ServoSolver 初始化.");
    pwm_x_ = 1500; pwm_y_ = 1500;
  }
  void run() override   { logger->info("ServoSolver 运行."); }
  void pause() override { logger->info("ServoSolver 暂停."); }
  void reset() override { logger->info("ServoSolver 重置."); }

  void on_hat(const std::string &data, fins::AcqTime) {
    auto v = parse_ints(data);
    if (v.size() >= 2 && py_bridge::solve_servo(v[0], v[1], pwm_x_, pwm_y_)) {
      send("servo", "1," + std::to_string(pwm_x_));
      send("servo", "2," + std::to_string(pwm_y_));
    }
  }
private:
  int pwm_x_, pwm_y_;
  static std::vector<int> parse_ints(const std::string &s) {
    std::vector<int> v;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, ','))
      if (!token.empty()) v.push_back(std::stoi(token));
    return v;
  }
};
EXPORT_NODE(ServoSolver)

// ──────────── GripperSolver ────────────
class GripperSolver : public fins::Node {
public:
  GripperSolver() = default;

  void define() override {
    set_name("GripperSolver");
    set_description("按钮 → 夹爪方向解算");
    set_category("JoystickSolver");
    register_input<std::string>("buttons", &GripperSolver::on_buttons);
    register_output<std::string>("gripper");
  }
  void initialize() override { logger->info("GripperSolver 初始化."); }
  void run() override       { logger->info("GripperSolver 运行."); }
  void pause() override     { logger->info("GripperSolver 暂停."); }
  void reset() override     { logger->info("GripperSolver 重置."); }

  void on_buttons(const std::string &data, fins::AcqTime) {
    std::vector<int> v;
    std::stringstream ss(data);
    std::string t;
    while (std::getline(ss, t, ','))
      if (!t.empty()) v.push_back(std::stoi(t));
    bool close = false, open = false;
    for (int b : v) { if (b == 4) close = true; if (b == 5) open = true; }
    send("gripper", std::to_string(py_bridge::solve_gripper(close, open)));
  }
};
EXPORT_NODE(GripperSolver)
