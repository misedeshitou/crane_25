#include "cmsis_os.h"

// behaviors
#include "behaviors/behavior_status.hpp"
#include "behaviors/behavior_tick.hpp"
#include "behaviors/calibrate/calibrate.hpp"
#include "behaviors/disable/disable.hpp"
#include "behaviors/store/store.hpp"
#include "hardwares/can/can.hpp"

//底盘是否运动到指定位置
bool chassis_done = false;

extern "C" void main_task()
{
  bool calibrated = false;
  while (true) {
    if (!calibrated)
      calibrated = (calibrate() == BehaviorStatus::SUCCESS);
    else {
      if (chassis_done)
        chassis_done = !(store(can::autoaim_data.layer) == BehaviorStatus::SUCCESS);
      else {
        disable();
      }
    }
    osDelay(BEHAVIOR_TICK_MS);
  }
}