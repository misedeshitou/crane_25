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

// -------------------- 机械参数 --------------------
constexpr float X_RADIUS = 2e-2;   // m
constexpr float Y_RADIUS = 1e-2;   // m
constexpr float Z_RADIUS = 45e-3;  // m

constexpr float MIN_X = 0;      // m
constexpr float MIN_Y = 0;      // m
constexpr float MIN_Z = -0.75;  // m

constexpr float MAX_X = 0.37;  // m
constexpr float MAX_Y = 0.23;  // m
constexpr float MAX_Z = 0;     // m

// -------------------- 对外功能 --------------------
extern float x;  // 只读! 单位: m
extern float y;  // 只读! 单位: m
extern float z;  // 只读! 单位: m

void disable_x();
void disable_y();
void disable_z();

void init_x();
void init_y();
void init_z();

void add_x(float value);
void add_y(float value);
void add_z(float value);

void cmd_x(float value);
void cmd_y(float value);
void cmd_z(float value);

void cmd_vx(float value);
void cmd_vy(float value);
void cmd_vz(float value);

void control_x();
void control_y();
void control_z();

// -------------------- 对外硬件 --------------------
extern sp::RM_Motor motor_xl;
extern sp::RM_Motor motor_xr;
extern sp::RM_Motor motor_yf;
extern sp::RM_Motor motor_yr;
extern sp::RM_Motor motor_zl;
extern sp::RM_Motor motor_zr;

// -------------------- 对外调试 --------------------
extern sp::PID x_pid;  // m -> m/s
extern sp::PID y_pid;  // m -> m/s
extern sp::PID z_pid;  // m -> m/s

extern sp::PID speed_xl_pid;  // rad/s -> N·m
extern sp::PID speed_xr_pid;  // rad/s -> N·m
extern sp::PID speed_yf_pid;  // rad/s -> N·m
extern sp::PID speed_yr_pid;  // rad/s -> N·m
extern sp::PID speed_zl_pid;  // rad/s -> N·m
extern sp::PID speed_zr_pid;  // rad/s -> N·m

}  // namespace frame
#endif