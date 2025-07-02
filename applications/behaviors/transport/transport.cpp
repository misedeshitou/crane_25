#include "transport.hpp"

#include "tools/math_tools/math_tools.hpp"

// behaviors
#include "behaviors/behavior_tick.hpp"

// hardwares
#include "hardwares/can/can.hpp"
#include "hardwares/chassis/chassis.hpp"

BehaviorStatus transport()
{
  if (
    (std::abs(can::autoaim_data.target_x - chassis::chassis_.odom_x) <= 0.01) &&
    (std::abs(can::autoaim_data.target_y - chassis::chassis_.odom_y) <= 0.01) &&
    (std::abs(can::autoaim_data.target_yaw - chassis::chassis_.odom_yaw) <= 0.01)) {
    return BehaviorStatus::SUCCESS;
  }
  else {
    chassis::control_drive();
    chassis::control_pivot();
    return BehaviorStatus::RUNNING;
  }
}
