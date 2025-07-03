#include "chassis.hpp"

#include "swerve_omni.hpp"
namespace chassis
{
HardwareMode chassis_mode = HardwareMode::DISABLE;

sp::Swerve_Omni chassis_(
  SWERVE_WHEEL_RADIUS, OMNI_WHEEL_RADIUS, CHASSIS_HALF_LENGTH, CHASSIS_HALF_WIDTH);

sp::RM_Motor omni_1(1, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor pivot_2(5, sp::RM_Motors::GM6020);
sp::RM_Motor drive_2(2, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor omni_3(3, sp::RM_Motors::M3508, sp::M3508_P19);
sp::RM_Motor pivot_4(6, sp::RM_Motors::GM6020);
sp::RM_Motor drive_4(4, sp::RM_Motors::M3508, sp::M3508_P19);

sp::PID omni_1_speed_pid(T, 1, 0, 0, 1, 1);

sp::PID pivot_2_pos_pid(T, 1, 0, 0, 1, 1);
sp::PID pivot_2_speed_pid(T, 1, 0, 0, 1, 1);
sp::PID drive_2_speed_pid(T, 1, 0, 0, 1, 1);

sp::PID omni_3_speed_pid(T, 1, 0, 0, 1, 1);

sp::PID pivot_4_pos_pid(T, 1, 1, 0, 5, 1);
sp::PID pivot_4_speed_pid(T, 1, 1, 0, 5, 1);
sp::PID drive_4_speed_pid(T, 1, 0, 0, 1, 1);

void disable_chassis() { chassis_mode = HardwareMode::DISABLE; }

void cmd_v(float vx, float vy, float wz)
{
  chassis_mode = HardwareMode::VELOCITY;
  chassis_.calc(vx, vy, wz, pivot_2.angle, pivot_4.angle);
}

void control_pivot()
{
  if (chassis_mode == HardwareMode::DISABLE) {
    pivot_2.cmd(0);
    pivot_4.cmd(0);
    return;
  }

  else if (chassis_mode == HardwareMode::VELOCITY) {
    pivot_2_pos_pid.calc(chassis_.angle_lr, pivot_2.angle);
    pivot_2_speed_pid.calc(chassis_.speed_lr, pivot_2.speed);
    pivot_2.cmd(pivot_2_speed_pid.out);

    pivot_4_pos_pid.calc(chassis_.angle_rf, pivot_4.angle);
    pivot_4_speed_pid.calc(chassis_.speed_rf, pivot_4.speed);
    pivot_4.cmd(pivot_4_speed_pid.out);
  }
}
void control_drive()
{
  if (chassis_mode == HardwareMode::DISABLE) {
    omni_1.cmd(0);
    drive_2.cmd(0);
    omni_3.cmd(0);
    drive_4.cmd(0);
    return;
  }

  else if (chassis_mode == HardwareMode::VELOCITY) {
    omni_1_speed_pid.calc(chassis_.speed_lf, omni_1.speed);
    omni_1.cmd(omni_1_speed_pid.out);
    drive_2_speed_pid.calc(chassis_.speed_lr, drive_2.speed);
    drive_2.cmd(drive_2_speed_pid.out);
    omni_3_speed_pid.calc(chassis_.speed_rr, omni_3.speed);
    omni_3.cmd(omni_3_speed_pid.out);
    drive_4_speed_pid.calc(chassis_.speed_rf, drive_4.speed);
    drive_4.cmd(drive_4_speed_pid.out);
  }
}

}  // namespace chassis