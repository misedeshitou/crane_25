#include "keyboard_control.hpp"

// behaviors
#include "behaviors/behavior_tick.hpp"
#include "behaviors/disable/disable.hpp"
#include "behaviors/store/store.hpp"

// hardwares
#include "hardwares/frame/frame.hpp"
#include "tools/math_tools/math_tools.hpp"

enum class ActionMode
{
  IDLE,
  STORE_LEFT,
};

sp::LowPassFilter custom_yaw_filter(0.01);
sp::LowPassFilter custom_roll1_filter(0.01);

static ActionMode action_mode = ActionMode::IDLE;

static void keyboard_control_fpv()
{
  fpv::add_z(vt02.mouse.vs * T * 10);        // VS_RATIO
  fpv::add_pitch(-vt02.mouse.vy * 500 * T);  // VY_RATIO
}

static void keyboard_control_gas()
{
  static int gas_left_cnt = 0;
  static int gas_right_cnt = 0;
  static int gas_arm_cnt = 0;
  static bool last_buttom = false;

  if (vt02.keys.z) gas_left_cnt++;
  if (vt02.keys.x) gas_right_cnt++;
  if (vt02.keys.c) gas_arm_cnt++;

  if (gas_arm_cnt > 0.5 / T) {
    gas_arm_cnt = 0;
    gas::arm = !gas::arm;
  }
  if (gas_left_cnt > 0.5 / T) {
    gas_left_cnt = 0;
    gas::left = !gas::left;
  }
  if (gas_right_cnt > 0.5 / T) {
    gas_right_cnt = 0;
    gas::right = !gas::right;
  }

  if (!last_buttom && vt02.custom.button) {
    gas::arm = !gas::arm;
  }
  last_buttom = vt02.custom.button;
}

static void keyboard_control_frame()
{
  static float last_custom_z = 0;

  frame::cmd_x(vt02.custom.x);
  frame::cmd_y(vt02.custom.y);

  frame::add_z(
    vt02.keys.r * frame::MAX_VZ * T - vt02.keys.f * frame::MAX_VZ * T + vt02.custom.z -
    last_custom_z);

  last_custom_z = vt02.custom.z;
}

static void keyboard_control_store()
{
  static int store_left_cnt = 0;

  if (vt02.keys.v) store_left_cnt++;
  if (store_left_cnt > 1 / T) {
    store_left_cnt = 0;
    action_mode = (action_mode == ActionMode::IDLE) ? ActionMode::STORE_LEFT : ActionMode::IDLE;
  }

  static bool stored_left = false;

  if (action_mode == ActionMode::STORE_LEFT) {
    if (!stored_left)
      stored_left = (store_left() == BehaviorStatus::SUCCESS);
    else
      action_mode = ActionMode::IDLE;
  }
  else {
    stored_left = false;
  }
}

void keyboard_control()
{
  if (action_mode != ActionMode::IDLE) {
    keyboard_control_fpv();
    keyboard_control_chassis();
    keyboard_control_gas();
    keyboard_control_store();
  }
  else {
    keyboard_control_fpv();
    keyboard_control_chassis();
    keyboard_control_frame();
    keyboard_control_arm();
    keyboard_control_gas();
    keyboard_control_store();
  }
}