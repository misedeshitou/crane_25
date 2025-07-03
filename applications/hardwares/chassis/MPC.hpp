#ifndef MPC_HPP
#define MPC_HPP

#include "hardwares/can/can.hpp"

constexpr int STATE_DIM = 3;    // 状态维度 (x, y, yaw)
constexpr int CONTROL_DIM = 3;  // 控制维度 (v_x, v_y, omega)

// === 模型预测控制接口 ===

// 设置目标状态（期望位置和朝向）
void mpc_set_target(const float * x_d);

// 设置状态权重 Q 和控制权重 R
void mpc_set_weights(const float * Q, const float * R);

// 获取当前控制量
void mpc_get_control(float * u);

// 解单步优化问题，输出控制量
void mpc_solve_step(float * out_u);

// === 系统参数设置接口 ===

// 设置系统矩阵 A 和 B
void mpc_set_system_matrices(const float * A, const float * B);

// 设置离散化时间步长
void mpc_set_time_step(float dt);

// === 全局变量声明 ===

extern float A[STATE_DIM * STATE_DIM];      // 状态转移矩阵
extern float B[STATE_DIM * CONTROL_DIM];    // 控制矩阵
extern float state_weights[STATE_DIM];      // 状态权重
extern float control_weights[CONTROL_DIM];  // 控制权重
extern float dt;                            // 离散化时间步长
extern can::autoaim_measure autoaim_data;   // 外部CAN数据接口

#endif  // MPC_HPP