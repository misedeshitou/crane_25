#include "store.hpp"

// behaviors
#include "behaviors/behavior_tick.hpp"
#include "behaviors/move/move.hpp"

// hardwares
#include "hardwares/frame/frame.hpp"

BehaviorStatus store(bool layer)
{
  static bool moved_stretch = false;
  static bool moved_z1 = false;
  static bool moved_x2 = false;
  static bool switched_claw_1 = false;
  static bool moved_z2 = false;
  static bool switched_claw_2 = false;
  static bool moved_z3 = false;

  //牛奶箱子高度0.12m
  if (!moved_stretch) {
    moved_stretch =
      (move(x_stretched, z_positionB - z_positionA, false) == BehaviorStatus::SUCCESS);
  }
  else if (!switched_claw_1) {
    switched_claw_1 = (switch_claw(true) == BehaviorStatus::SUCCESS);
  }
  else if (!moved_z1) {
    if (!layer)
      moved_z1 = (move_z(z_positionA - z_positionB) == BehaviorStatus::SUCCESS);
    else
      moved_z1 = (move_z(z_positionD - z_positionB) == BehaviorStatus::SUCCESS);
  }
  else if (!moved_x2) {
    moved_x2 = (move_x(-x_stretched) == BehaviorStatus::SUCCESS);
  }
  else if (!moved_z2) {
    if (!layer)
      moved_z2 = (move_z(z_positionC - z_positionA) == BehaviorStatus::SUCCESS);
    else
      moved_z2 = (move_z(z_positionE - z_positionD) == BehaviorStatus::SUCCESS);
  }
  else if (!switched_claw_2) {
    switched_claw_2 = (switch_claw(false) == BehaviorStatus::SUCCESS);
  }
  else if (!moved_z3) {
    if (!layer)
      moved_z3 = (move_z(z_positionA - z_positionC) == BehaviorStatus::SUCCESS);
    else
      moved_z3 = (move_z(z_positionD - z_positionE) == BehaviorStatus::SUCCESS);
  }
  else {
    moved_stretch = false;
    moved_z1 = false;
    moved_x2 = false;
    switched_claw_1 = false;
    moved_z2 = false;
    switched_claw_2 = false;
    moved_z3 = false;
    return BehaviorStatus::SUCCESS;
  }

  return BehaviorStatus::RUNNING;
}
