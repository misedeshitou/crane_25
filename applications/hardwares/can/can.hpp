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
  bool action;  // 0为抓取，1为放置
  bool layer;   // 0为低层，1为高层
  float target_x;
  float target_y;
  float target_yaw;
} autoaim_measure;

extern autoaim_measure autoaim_data;
void start();

void send_pivot();
void send_drive();

void send_frame_z();
void send_frame_x();
void autoaim_read(autoaim_measure * autoaim_state, uint8_t rx_data[8]);
}  // namespace can

#endif  // CAN_HPP