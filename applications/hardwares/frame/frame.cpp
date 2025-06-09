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

// sp::RM_Motor motor_xl(1, sp::RM_Motors::M2006, sp::M2006_P36);
// sp::RM_Motor motor_xr(2, sp::RM_Motors::M2006, sp::M2006_P36);
// sp::RM_Motor motor_yf(3, sp::RM_Motors::M2006, sp::M2006_P36);
// sp::RM_Motor motor_yr(4, sp::RM_Motors::M2006, sp::M2006_P36);
// sp::RM_Motor motor_zl(5, sp::RM_Motors::M3508, sp::M3508_P19);
// sp::RM_Motor motor_zr(6, sp::RM_Motors::M3508, sp::M3508_P19);

sp::PID x_pid(T, 5, 0, 0, MAX_VX, MAX_VX);
sp::PID z_pid(T, 5, 0, 0, MAX_VZ, MAX_VZ);

sp::PID speed_xl_pid(T, 1, 0, 0, 1, 1);
sp::PID speed_xr_pid(T, 1, 0, 0, 1, 1);
sp::PID speed_zl_pid(T, 1, 1, 0, 5, 1);
sp::PID speed_zr_pid(T, 1, 1, 0, 5, 1);

sp::MotorComposer motor_x(false, true);
sp::MotorComposer motor_z(true, false);

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

void init_x() { motor_x.init_angles(motor_xl.angle, motor_xr.angle); }
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
  motor_x.calc_angle(motor_xl.angle, motor_xr.angle);
  x = motor_x.angle * X_RADIUS;

  if (x_mode == HardwareMode::DISABLE) {
    motor_xl.cmd(0);
    motor_xr.cmd(0);
    return;
  }

  if (x_mode == HardwareMode::POSITION) {
    x_pid.calc(x_set, x);
    motor_x.calc_speeds(x_pid.out / X_RADIUS);
  }
  else if (x_mode == HardwareMode::VELOCITY) {
    motor_x.calc_speeds(vx_set / X_RADIUS);
  }

  speed_xl_pid.calc(motor_x.speed_l, motor_xl.speed);
  speed_xr_pid.calc(motor_x.speed_r, motor_xr.speed);
  motor_xl.cmd(speed_xl_pid.out);
  motor_xr.cmd(speed_xr_pid.out);
}

void control_z()
{
  motor_z.calc_angle(motor_zl.angle, motor_zr.angle);
  z = motor_z.angle * Z_RADIUS;

  if (z_mode == HardwareMode::DISABLE) {
    motor_zl.cmd(0);
    motor_zr.cmd(0);
    return;
  }

  if (z_mode == HardwareMode::POSITION) {
    z_pid.calc(z_set, z);
    motor_z.calc_speeds(z_pid.out / Z_RADIUS);
  }
  else if (z_mode == HardwareMode::VELOCITY) {
    motor_z.calc_speeds(vz_set / Z_RADIUS);
  }

  speed_zl_pid.calc(motor_z.speed_l, motor_zl.speed);
  speed_zr_pid.calc(motor_z.speed_r, motor_zr.speed);
  motor_zl.cmd(speed_zl_pid.out);
  motor_zr.cmd(speed_zr_pid.out);
}

}  // namespace frame