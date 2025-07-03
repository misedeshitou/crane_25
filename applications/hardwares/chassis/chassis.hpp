#ifndef CHASSIS_HPP
#define CHASSIS_HPP

#include "control_task.hpp"
#include "hardwares/chassis/swerve_omni.hpp"
#include "hardwares/hardware_mode.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/motor_composer/motor_composer.hpp"
#include "tools/pid/pid.hpp"

namespace chassis
{
// -------------------- 控制参数 --------------------
constexpr float T = CONTROL_TICK_MS / 1e3;  // s

static const float CHASSIS_HALF_LENGTH = 0.300f;
static const float CHASSIS_HALF_WIDTH = 0.150f;
static const float SWERVE_WHEEL_RADIUS = 0.055f;
static const float OMNI_WHEEL_RADIUS = 0.065f;

constexpr float MAX_VX = 0.5;  // m/s
constexpr float MAX_VY = 0.5;  // m/s
constexpr float MAX_VZ = 0.5;  // m/s

// -------------------- 对外功能 --------------------

void disable_chassis();
void cmd_v(float vx, float vy, float wz);
void control_pivot();
void control_drive();
// -------------------- 对外硬件 --------------------
extern sp::Swerve_Omni chassis_;

extern sp::RM_Motor omni_1;
extern sp::RM_Motor pivot_2;
extern sp::RM_Motor drive_2;
extern sp::RM_Motor omni_3;
extern sp::RM_Motor pivot_4;
extern sp::RM_Motor drive_4;

// -------------------- 对外调试 --------------------
extern sp::PID omni_1_speed_pid;

extern sp::PID pivot_2_pos_pid;
extern sp::PID pivot_2_speed_pid;
extern sp::PID drive_2_speed_pid;

extern sp::PID omni_3_speed_pid;

extern sp::PID pivot_4_pos_pid;
extern sp::PID pivot_4_speed_pid;
extern sp::PID drive_4_speed_pid;

}  // namespace chassis
#endif