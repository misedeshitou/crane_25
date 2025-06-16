#include "frame.hpp"

namespace frame
{
float x = 0;
float z = 0;

HardwareMode x_mode = HardwareMode::DISABLE;
HardwareMode z_mode = HardwareMode::DISABLE;

float x_set = 0;
float z_set = 0;
float vx_set = 0;
float vz_set = 0;

sp::RM_Motor motor_x1(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_x2(2, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_x3(3, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_x4(4, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_zl(5, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor motor_zr(6, sp::RM_Motors::M3508, sp::M3508_P19);

sp::PID x_pid(T, 5, 0, 0, MAX_VX, MAX_VX);
sp::PID z_pid(T, 5, 0, 0, MAX_VZ, MAX_VZ);

sp::PID speed_x1_pid(T, 1, 0, 0, 1, 1);
sp::PID speed_x2_pid(T, 1, 0, 0, 1, 1);
sp::PID speed_x3_pid(T, 1, 0, 0, 1, 1);
sp::PID speed_x4_pid(T, 1, 0, 0, 1, 1);
sp::PID speed_zl_pid(T, 1, 1, 0, 5, 1);
sp::PID speed_zr_pid(T, 1, 1, 0, 5, 1);

sp::MotorComposer motor_x(false, true);
sp::MotorComposer motor_z(false, false);

void disable_x()
{
  x_mode = HardwareMode::DISABLE;
  x_set = x;
}

void disable_z()
{
  z_mode = HardwareMode::DISABLE;
  z_set = z;
}

void init_x()
{
  motor_x.init_angles(motor_x1.angle, motor_x3.angle);
  motor_x.init_angles(motor_x2.angle, motor_x4.angle);
}
void init_z() { motor_z.init_angles(motor_zl.angle, motor_zr.angle); }

void add_x(float value) { cmd_x(x_set + value); }

void add_z(float value) { cmd_z(z_set + value); }

void cmd_x(float value)
{
  x_mode = HardwareMode::POSITION;
  x_set = sp::limit_min_max(value, MIN_X, MAX_X);
}

void cmd_z(float value)
{
  z_mode = HardwareMode::POSITION;
  z_set = sp::limit_min_max(value, MIN_Z, MAX_Z);
}

void cmd_vx(float value)
{
  x_mode = HardwareMode::VELOCITY;
  vx_set = value;
}

void cmd_vz(float value)
{
  z_mode = HardwareMode::VELOCITY;
  vz_set = value;
}

void control_x()
{
  motor_x.calc_angle(motor_x1.angle, motor_x3.angle);
  x = motor_x.angle / Xdistance_to_Xangle;

  if (x_mode == HardwareMode::DISABLE) {
    motor_x1.cmd(0);
    motor_x2.cmd(0);
    motor_x3.cmd(0);
    motor_x4.cmd(0);
    return;
  }

  if (x_mode == HardwareMode::POSITION) {
    x_pid.calc(x_set, x);
    motor_x.calc_speeds(x_pid.out * Xdistance_to_Xangle);
  }
  else if (x_mode == HardwareMode::VELOCITY) {
    motor_x.calc_speeds(vx_set * Xdistance_to_Xangle);
  }
  speed_x1_pid.calc(motor_x.speed_l, motor_x1.speed);
  speed_x2_pid.calc(motor_x.speed_r, motor_x2.speed);
  speed_x3_pid.calc(motor_x.speed_l, motor_x3.speed);
  speed_x4_pid.calc(motor_x.speed_r, motor_x4.speed);
  motor_x1.cmd(speed_x1_pid.out);
  motor_x2.cmd(speed_x2_pid.out);
  motor_x3.cmd(speed_x3_pid.out);
  motor_x4.cmd(speed_x4_pid.out);
}

void control_z()
{
  motor_z.calc_angle(motor_zl.angle, motor_zr.angle);
  z = motor_z.angle / Zdistance_to_Zangle;

  if (z_mode == HardwareMode::DISABLE) {
    motor_zl.cmd(0);
    motor_zr.cmd(0);
    return;
  }

  if (z_mode == HardwareMode::POSITION) {
    z_pid.calc(z_set, z);
    motor_z.calc_speeds(z_pid.out * Zdistance_to_Zangle);
  }
  else if (z_mode == HardwareMode::VELOCITY) {
    motor_z.calc_speeds(vz_set * Zdistance_to_Zangle);
  }

  speed_zl_pid.calc(motor_z.speed_l, motor_zl.speed);
  speed_zr_pid.calc(motor_z.speed_r, motor_zr.speed);
  motor_zl.cmd(speed_zl_pid.out);
  motor_zr.cmd(speed_zr_pid.out);
}

}  // namespace frame