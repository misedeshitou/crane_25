#include "control_task.hpp"

#include "cmsis_os.h"

// behaviors
#include "behaviors/behavior_tick.hpp"
#include "hardwares/can/can.hpp"
#include "hardwares/frame/frame.hpp"

extern "C" void control_task()
{
  can::start();
  while (true) {
    frame::control_z();
    can::send_frame_z();

    frame::control_x();
    can::send_frame_x();

    osDelay(BEHAVIOR_TICK_MS);
  }
}