#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <cstring>

constexpr int STATE_SIZE = 6;  // 状态向量维度
constexpr int OBS_SIZE = 5;    // 观测向量维度

// === 卡尔曼滤波器接口 ===

// 初始化卡尔曼滤波器
void kalman_init(const float * x0, const float * P0);

// 预测步骤
void kalman_predict();

// 更新步骤
void kalman_update(const float * z);

// 获取当前状态
float * kalman_get_state();

// === 系统矩阵设置接口 ===
// 在初始化前直接对全局 F, Q, H, R 赋值

extern float F[STATE_SIZE * STATE_SIZE];  // 状态转移矩阵
extern float Q[STATE_SIZE * STATE_SIZE];  // 过程噪声协方差
extern float H[OBS_SIZE * STATE_SIZE];    // 观测矩阵
extern float R[OBS_SIZE * OBS_SIZE];      // 观测噪声协方差

// === 矩阵运算工具函数 ===
// 这些函数在卡尔曼滤波器的实现中使用，通常不需要直接调用

extern void matMul(const float * A, int ar, int ac, const float * B, int br, int bc, float * C);
extern void matAdd(const float * A, const float * B, int r, int c, float * C);
extern void matSub(const float * A, const float * B, int r, int c, float * C);
extern void matCopy(const float * A, int r, int c, float * B);
extern void matIdentity(float * A, int n);
extern int matInv5x5(const float * A, float * A_inv);

#endif  // KALMAN_FILTER_HPP