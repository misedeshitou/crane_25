#include "cmsis_os.h"

// behaviors
#include "behaviors/behavior_status.hpp"
#include "behaviors/behavior_tick.hpp"
#include "behaviors/calibrate/calibrate.hpp"

extern "C" void main_task()
{
  bool calibrated = false;
  while (true) {
    // if (!calibrated)
    //   calibrated = (calibrate() == BehaviorStatus::SUCCESS);
    // else
    //   auto_control();

    osDelay(BEHAVIOR_TICK_MS);
  }
}