#include "calibrate.hpp"

// behaviors
#include "behaviors/behavior_tick.hpp"
#include "behaviors/disable/disable.hpp"
#include "behaviors/move/move.hpp"

// hardwares
#include "hardwares/frame/frame.hpp"

static BehaviorStatus calibrate_x()
{
  static int reach_cnt = 0;

  frame::cmd_vx(-0.1);

  if (std::abs(frame::motor_xl.torque) > 0.5 || std::abs(frame::motor_xr.torque) > 0.5) reach_cnt++;
  if (reach_cnt < 0.5 / T) return BehaviorStatus::RUNNING;

  frame::init_x();
  frame::cmd_vx(0);

  reach_cnt = 0;
  return BehaviorStatus::SUCCESS;
}

static BehaviorStatus calibrate_z()
{
  static int reach_cnt = 0;

  frame::cmd_vz(0.5);

  if (std::abs(frame::motor_zl.torque) > 2 || std::abs(frame::motor_zr.torque) > 2) reach_cnt++;
  if (reach_cnt < 0.5 / T) return BehaviorStatus::RUNNING;

  frame::init_z();
  frame::cmd_vz(0);

  reach_cnt = 0;
  return BehaviorStatus::SUCCESS;
}

BehaviorStatus calibrate()
{
  static bool calibrated_x = false;
  static bool moved_x = false;
  static bool calibrated_z = false;

  if (!calibrated_x)
    calibrated_x = (calibrate_x() == BehaviorStatus::SUCCESS);

  else if (!moved_x)
    moved_x = (move_x(frame::MAX_X / 2) == BehaviorStatus::SUCCESS);

  else if (!calibrated_z)
    calibrated_z = (calibrate_z() == BehaviorStatus::SUCCESS);

  else {
    calibrated_x = false;
    moved_x = false;
    calibrated_z = false;
    return BehaviorStatus::SUCCESS;
  }

  return BehaviorStatus::RUNNING;
}