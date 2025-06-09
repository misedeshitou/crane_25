#include "cmsis_os.h"

// behaviors
#include "behaviors/behavior_tick.hpp"


extern "C" void main_task()
{

  while (true) {
    //   remote_control();

    osDelay(BEHAVIOR_TICK_MS);
  }
}