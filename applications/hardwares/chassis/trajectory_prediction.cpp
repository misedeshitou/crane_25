#include "trajectory_prediction.hpp"

#include "MPC.hpp"
#include "chassis.hpp"
#include "cmsis_os.h"
#include "kalman_filter.hpp"
#include "swerve_omni.hpp"

extern float ins_gyro[3];
extern "C" void trajectory_prediction_task()
{
  // === 初始化系统时钟、硬件（略） ===
  // 设置 F（状态转移矩阵），假设采样周期 dt = 0.01s
  float dt = 0.01f;
  matIdentity(F, STATE_SIZE);  // 先单位阵
  F[0 * STATE_SIZE + 3] = dt;  // x+=vx*dt
  F[1 * STATE_SIZE + 4] = dt;  // y+=vy*dt
  F[2 * STATE_SIZE + 5] = dt;  // theta+=omega*dt

  // 初始化观测矩阵 H
  memset(H, 0, sizeof(H));
  H[0 * STATE_SIZE + 0] = 1;  // x
  H[1 * STATE_SIZE + 1] = 1;  // y
  H[2 * STATE_SIZE + 3] = 1;  // vx
  H[3 * STATE_SIZE + 4] = 1;  // vy
  H[4 * STATE_SIZE + 5] = 1;  // omega

  // 初始化 Q 和 R（对角线为小值），可根据传感器噪声调整
  matIdentity(Q, STATE_SIZE);
  for (int i = 0; i < STATE_SIZE; i++) Q[i * STATE_SIZE + i] = 1e-4;
  matIdentity(R, OBS_SIZE);
  for (int i = 0; i < OBS_SIZE; i++) R[i * OBS_SIZE + i] = 1e-3;

  // 初始化状态与协方差
  float x0[STATE_SIZE] = {0};
  float P0[STATE_SIZE * STATE_SIZE] = {0};
  for (int i = 0; i < STATE_SIZE; i++) {
    P0[i * STATE_SIZE + i] = 1e-1;
  }
  kalman_init(x0, P0);

  // MPC 初始化
  float mpc_state_weights[STATE_DIM] = {10, 10, 5};
  float mpc_control_weights[CONTROL_DIM] = {0.1, 0.1, 0.1};
  mpc_set_weights(mpc_state_weights, mpc_control_weights);

  // === 主循环 ===
  while (1) {
    // 1. 预测
    kalman_predict();

    // 2. 获取传感器数据
    float z[OBS_SIZE] = {0};
    z[0] = chassis::chassis_.odom_x;
    z[1] = chassis::chassis_.odom_y;
    z[2] = chassis::chassis_.vx;
    z[3] = chassis::chassis_.vy;
    z[4] = ins_gyro[2];  // 假设 ins_gyro[2] 是角速度

    // 3. 更新滤波器
    kalman_update(z);

    // 4. 获取滤波后的状态
    const float * state = kalman_get_state();

    // === 使用滤波结果进行MPC控制 ===

    float target_state[STATE_DIM] = {
      can::autoaim_data.target_x, can::autoaim_data.target_y, can::autoaim_data.target_yaw};
    mpc_set_target(target_state);

    float control[CONTROL_DIM] = {state[0], state[1], state[2]};
    mpc_solve_step(control);

    // 发送控制信号到底盘
    chassis::cmd_v(control[0], control[1], control[2]);

    // 延时
    osDelay(10);
  }
}