#ifndef FRAME_HPP
#define FRAME_HPP

#include "control_task.hpp"
#include "hardwares/hardware_mode.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/motor_composer/motor_composer.hpp"
#include "tools/pid/pid.hpp"

namespace frame
{
// -------------------- 控制参数 --------------------
constexpr float T = CONTROL_TICK_MS / 1e3;  // s

constexpr float MAX_VX = 0.5;  // m/s
constexpr float MAX_VY = 0.5;  // m/s
constexpr float MAX_VZ = 0.5;  // m/s

constexpr float Zdistance_to_Zangle = 2 * sp::PI / 0.01;  // rad/m
constexpr float Xdistance_to_Xangle = 2 * sp::PI / 0.1;   // rad/m

// -------------------- 机械参数 --------------------
constexpr float MIN_X = 0;      // m
constexpr float MIN_Y = 0;      // m
constexpr float MIN_Z = -0.75;  // m

constexpr float MAX_X = 0.37;  // m
constexpr float MAX_Y = 0.23;  // m
constexpr float MAX_Z = 0;     // m

// -------------------- 对外功能 --------------------
extern float x;  // 只读! 单位: m
extern float z;  // 只读! 单位: m

void disable_x();
void disable_z();

void init_x();
void init_z();

void add_x(float value);
void add_z(float value);

void cmd_x(float value);
void cmd_z(float value);

void cmd_vx(float value);
void cmd_vz(float value);

void control_x();
void control_z();

// -------------------- 对外硬件 --------------------
extern sp::RM_Motor motor_x1;
extern sp::RM_Motor motor_x2;
extern sp::RM_Motor motor_x3;
extern sp::RM_Motor motor_x4;
extern sp::RM_Motor motor_zl;
extern sp::RM_Motor motor_zr;

// -------------------- 对外调试 --------------------
extern sp::PID x_pid;  // m -> m/s
extern sp::PID y_pid;  // m -> m/s
extern sp::PID z_pid;  // m -> m/s

extern sp::PID speed_xl_pid;  // rad/s -> N·m
extern sp::PID speed_xr_pid;  // rad/s -> N·m
extern sp::PID speed_zl_pid;  // rad/s -> N·m
extern sp::PID speed_zr_pid;  // rad/s -> N·m

}  // namespace frame
#endif