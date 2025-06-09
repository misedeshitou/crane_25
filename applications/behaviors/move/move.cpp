#include "move.hpp"

#include "tools/math_tools/math_tools.hpp"

// behaviors
#include "behaviors/behavior_tick.hpp"

// hardwares
#include "hardwares/frame/frame.hpp"
#include "hardwares/servo/servo.hpp"

BehaviorStatus move_x(float x)
{
  static int reach_cnt = 0;

  // frame::x_mode = HardwareMode::POSITION;
  frame::cmd_x(x);

  if (std::abs(frame::x - x) < DIST_EPS) reach_cnt++;
  if (reach_cnt < 0.5 / T) return BehaviorStatus::RUNNING;

  reach_cnt = 0;
  return BehaviorStatus::SUCCESS;
}

BehaviorStatus move_z(float z)
{
  static int reach_cnt = 0;

  // frame::z_mode = HardwareMode::POSITION;
  frame::cmd_z(z);

  if (std::abs(frame::z - z) < DIST_EPS) reach_cnt++;
  if (reach_cnt < 0.5 / T) return BehaviorStatus::RUNNING;

  reach_cnt = 0;
  return BehaviorStatus::SUCCESS;
}

BehaviorStatus switch_claw(bool claw)
{
  if (claw) {
    servo::claw_close();
  }
  else {
    servo::claw_open();
  }
  return BehaviorStatus::SUCCESS;
}

BehaviorStatus move(float x, float z, bool claw)
{
  static bool moved_x = false;
  static bool moved_z = false;
  static bool switched_claw = false;
  if (!moved_x) moved_x = (move_x(x) == BehaviorStatus::SUCCESS);
  if (!moved_z) moved_z = (move_z(z) == BehaviorStatus::SUCCESS);
  if (!switched_claw) switched_claw = (switch_claw(claw) == BehaviorStatus::SUCCESS);

  moved_x = false;
  moved_z = false;
  return BehaviorStatus::SUCCESS;
}
