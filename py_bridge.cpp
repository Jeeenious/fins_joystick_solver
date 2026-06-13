/*******************************************************************************
 * py_bridge.cpp — pybind11::embed 实现
 ******************************************************************************/

#include "py_bridge.hpp"
#include <pybind11/embed.h>
#include <pybind11/stl.h>

namespace py = pybind11;

void py_bridge::init() {
  static bool inited = false;
  if (inited) return;
  inited = true;

  py::initialize_interpreter();

  py::module_ sys = py::module_::import("sys");
  py::list path = sys.attr("path");
  path.attr("insert")(0, PY_PROJECT_DIR);

  try {
    py::module_::import("solver_py.solver");
    py::print("[py_bridge] solver_py 导入成功");
  } catch (py::error_already_set &e) {
    py::print("[py_bridge] Python 导入失败:", e.what());
    throw;
  }
}

std::string py_bridge::solve_motion(float lh, float lv, float rh, float rv) {
  init();
  py::module_ m = py::module_::import("solver_py.solver");
  py::dict r = m.attr("solve_motion")(lh, lv, rh, rv);
  return std::to_string(r["x_pct"].cast<int>()) + "," +
         std::to_string(r["y_pct"].cast<int>()) + "," +
         std::to_string(r["yaw_rate_cmd"].cast<int>()) + "," +
         std::to_string(r["depth_force_cmd"].cast<int>());
}

bool py_bridge::solve_servo(int hat_x, int hat_y, int &pwm_x, int &pwm_y) {
  init();
  py::module_ m = py::module_::import("solver_py.solver");
  py::tuple r = m.attr("solve_servo")(hat_x, hat_y, pwm_x, pwm_y);
  pwm_x = r[0].cast<int>();
  pwm_y = r[1].cast<int>();
  return r[2].cast<bool>();
}

int py_bridge::solve_gripper(bool close_btn, bool open_btn) {
  init();
  py::module_ m = py::module_::import("solver_py.solver");
  return m.attr("solve_gripper")(close_btn, open_btn).cast<int>();
}
