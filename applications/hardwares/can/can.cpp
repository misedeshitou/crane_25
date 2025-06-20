#include "can.hpp"

#include "cmsis_os.h"

// hardwares
#include "hardwares/frame/frame.hpp"

namespace can
{
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
autoaim_measure autoaim_data;

void start()
{
  can1.config();
  can2.config();
  can1.start();
  can2.start();
}

void send_chassis()
{
  //   chassis::motor_lf.write(can2.tx_data);
  //   chassis::motor_lr.write(can2.tx_data);
  //   chassis::motor_rr.write(can2.tx_data);
  //   chassis::motor_rf.write(can2.tx_data);
  //   can2.send(chassis::motor_lf.tx_id);
}

void send_frame_z()
{
  frame::motor_x1.write(can1.tx_data);
  frame::motor_x2.write(can1.tx_data);
  frame::motor_x3.write(can1.tx_data);
  frame::motor_x4.write(can1.tx_data);
  can1.send(frame::motor_x1.tx_id);
}

void send_frame_x()
{
  frame::motor_zl.write(can1.tx_data);
  frame::motor_zr.write(can1.tx_data);
  can1.send(frame::motor_zl.tx_id);
}

void autoaim_read(autoaim_measure * autoaim_state, uint8_t rx_data[8])
{
  autoaim_state->action = rx_data[0];
  autoaim_state->layer = rx_data[1];
}

}  // namespace can