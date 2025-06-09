// #include "can_recv.hpp"

#include "can.hpp"
#include "cmsis_os.h"
constexpr uint32_t CAN_AUTOAIM_ID = 0x0FF;

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can::can1.recv();
      //   if (can1.rx_id == fric_motor1.rx_id) {
      //     fric_motor1.read(can1.rx_data, stamp_ms);
      //   }
      //   else if (can1.rx_id == fric_motor2.rx_id) {
      //     fric_motor2.read(can1.rx_data, stamp_ms);
      //   }
      //   else if (can1.rx_id == trigger_motor.rx_id)
      //     trigger_motor.read(can1.rx_data, stamp_ms);
      //   else if (can1.rx_id == pitch_motor.rx_id)
      //     pitch_motor.read(can1.rx_data, stamp_ms);
      //   //自瞄ID
      if (can::can1.rx_id == CAN_AUTOAIM_ID) {
        can::autoaim_read(&can::autoaim_data, can::can1.rx_data);
      }
    }
    if (hcan == &hcan2) {
      can::can2.recv();
      //   if (can2.rx_id == wheel_lf.rx_id)
      //     wheel_lf.read(can2.rx_data, stamp_ms);
      //   else if (can2.rx_id == wheel_lr.rx_id)
      //     wheel_lr.read(can2.rx_data, stamp_ms);
      //   else if (can2.rx_id == wheel_rf.rx_id)
      //     wheel_rf.read(can2.rx_data, stamp_ms);
      //   else if (can2.rx_id == wheel_rr.rx_id)
      //     wheel_rr.read(can2.rx_data, stamp_ms);
      //   else if (can2.rx_id == yaw_motor.rx_id)
      //     yaw_motor.read(can2.rx_data, stamp_ms);
      //   else if (can2.rx_id == CAN_CAP_ID)
      //     super_cap.read(can2.rx_data, stamp_ms);
    }
  }
}