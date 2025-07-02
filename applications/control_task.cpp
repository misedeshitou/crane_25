#include "control_task.hpp"

#include "cmsis_os.h"

// behaviors
#include "behaviors/behavior_tick.hpp"
#include "hardwares/can/can.hpp"
#include "hardwares/chassis/chassis.hpp"
#include "hardwares/frame/frame.hpp"
extern "C" void control_task()
{
  can::start();
  while (true) {
    frame::control_z();
    can::send_frame_z();

    frame::control_x();
    can::send_frame_x();

    chassis::control_pivot();
    can::send_pivot();

    chassis::control_drive();
    can::send_drive();

    osDelay(BEHAVIOR_TICK_MS);
  }
}