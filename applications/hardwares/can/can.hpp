#ifndef CAN_HPP
#define CAN_HPP

#include "io/can/can.hpp"

namespace can
{
extern sp::CAN can1;
extern sp::CAN can2;
//自瞄数据
typedef struct
{
  char vision_state;
  bool layer;  // 是否为上层
} autoaim_measure;

extern autoaim_measure autoaim_data;
void start();

void send_chassis();
void send_frame_z();
void send_frame_x();
void autoaim_read(autoaim_measure * autoaim_state, uint8_t rx_data[8]);
}  // namespace can

#endif  // CAN_HPP