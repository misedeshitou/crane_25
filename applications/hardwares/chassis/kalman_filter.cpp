#include <string.h>

#define STATE_SIZE 6
#define OBS_SIZE 5

// 状态向量与协方差
static float x[STATE_SIZE];               // 当前状态
static float P[STATE_SIZE * STATE_SIZE];  // 当前协方差

// 系统模型矩阵
float F[STATE_SIZE * STATE_SIZE];  // 状态转移
float Q[STATE_SIZE * STATE_SIZE];  // 过程噪声
float H[OBS_SIZE * STATE_SIZE];    // 观测矩阵
float R[OBS_SIZE * OBS_SIZE];      // 观测噪声

// 临时缓冲
static float tmp1[STATE_SIZE * STATE_SIZE];
static float tmp2[STATE_SIZE * STATE_SIZE];
static float tmp3[OBS_SIZE * STATE_SIZE];
static float tmpObs[OBS_SIZE * OBS_SIZE];
static float K[STATE_SIZE * OBS_SIZE];

// ==== 基本矩阵运算工具函数 ====
static void matMul(const float * A, int ar, int ac, const float * B, int br, int bc, float * C)
{
  // A: ar x ac, B: br x bc, must ac==br
  for (int i = 0; i < ar; i++) {
    for (int j = 0; j < bc; j++) {
      float sum = 0;
      for (int k = 0; k < ac; k++) {
        sum += A[i * ac + k] * B[k * bc + j];
      }
      C[i * bc + j] = sum;
    }
  }
}

static void matAdd(const float * A, const float * B, int r, int c, float * C)
{
  for (int i = 0; i < r * c; i++) {
    C[i] = A[i] + B[i];
  }
}

static void matSub(const float * A, const float * B, int r, int c, float * C)
{
  for (int i = 0; i < r * c; i++) {
    C[i] = A[i] - B[i];
  }
}

static void matCopy(const float * A, int r, int c, float * B)
{
  memcpy(B, A, r * c * sizeof(float));
}

void matIdentity(float * A, int n)
{
  memset(A, 0, n * n * sizeof(float));
  for (int i = 0; i < n; i++) {
    A[i * n + i] = 1;
  }
}

// Gauss-Jordan求逆矩阵（专门给 5x5 用，这里 OBS_SIZE=5）
static int matInv5x5(const float * A, float * A_inv)
{
  float aug[OBS_SIZE * (OBS_SIZE * 2)];
  int n = OBS_SIZE;
  // 初始化增广矩阵 [A|I]
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      aug[i * (2 * n) + j] = A[i * n + j];
    }
    for (int j = 0; j < n; j++) {
      aug[i * (2 * n) + n + j] = (i == j) ? 1.0f : 0.0f;
    }
  }
  // 高斯消元
  for (int i = 0; i < n; i++) {
    float pivot = aug[i * (2 * n) + i];
    if (pivot == 0.0f) return -1;
    for (int j = 0; j < 2 * n; j++) aug[i * (2 * n) + j] /= pivot;
    for (int k = 0; k < n; k++) {
      if (k == i) continue;
      float f = aug[k * (2 * n) + i];
      for (int j = 0; j < 2 * n; j++) {
        aug[k * (2 * n) + j] -= f * aug[i * (2 * n) + j];
      }
    }
  }
  // 提取逆
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      A_inv[i * n + j] = aug[i * (2 * n) + n + j];
    }
  }
  return 0;
}

// ==================== 卡尔曼滤波流程 ====================

// 初始化滤波器
void kalman_init(const float * x0, const float * P0)
{
  matCopy(x0, STATE_SIZE, 1, x);
  matCopy(P0, STATE_SIZE, STATE_SIZE, P);
}

// 预测步骤
void kalman_predict(void)
{
  // x = F*x
  float x_new[STATE_SIZE] = {0};
  matMul(F, STATE_SIZE, STATE_SIZE, x, STATE_SIZE, 1, x_new);
  matCopy(x_new, STATE_SIZE, 1, x);

  // P = F*P*F' + Q
  matMul(F, STATE_SIZE, STATE_SIZE, P, STATE_SIZE, STATE_SIZE, tmp1);  // F*P
  // F' (转置)
  float Ft[STATE_SIZE * STATE_SIZE] = {0};
  for (int i = 0; i < STATE_SIZE; i++) {
    for (int j = 0; j < STATE_SIZE; j++) {
      Ft[i * STATE_SIZE + j] = F[j * STATE_SIZE + i];
    }
  }
  matMul(tmp1, STATE_SIZE, STATE_SIZE, Ft, STATE_SIZE, STATE_SIZE, tmp2);  // F*P*F'
  matAdd(tmp2, Q, STATE_SIZE, STATE_SIZE, P);                              // + Q
}

// 更新步骤
// z 是观测向量，维度 OBS_SIZE
void kalman_update(const float * z)
{
  // y = z - H*x
  float Hx[OBS_SIZE] = {0};
  matMul(H, OBS_SIZE, STATE_SIZE, x, STATE_SIZE, 1, Hx);
  float y[OBS_SIZE] = {0};
  for (int i = 0; i < OBS_SIZE; i++) {
    y[i] = z[i] - Hx[i];
  }

  // S = H*P*H' + R
  matMul(H, OBS_SIZE, STATE_SIZE, P, STATE_SIZE, STATE_SIZE, tmp3);  // H*P
  float Ht[STATE_SIZE * OBS_SIZE] = {0};
  for (int i = 0; i < OBS_SIZE; i++) {
    for (int j = 0; j < STATE_SIZE; j++) {
      Ht[j * OBS_SIZE + i] = H[i * STATE_SIZE + j];  // H'为6x5
    }
  }
  matMul(tmp3, OBS_SIZE, STATE_SIZE, Ht, STATE_SIZE, OBS_SIZE, tmpObs);  // H*P*H'
  matAdd(tmpObs, R, OBS_SIZE, OBS_SIZE, tmpObs);                         // + R

  // K = P*H'*S^-1
  float S_inv[OBS_SIZE * OBS_SIZE] = {0};
  matInv5x5(tmpObs, S_inv);                                           // S^-1
  matMul(P, STATE_SIZE, STATE_SIZE, Ht, STATE_SIZE, OBS_SIZE, tmp1);  // P*H'
  matMul(tmp1, STATE_SIZE, OBS_SIZE, S_inv, OBS_SIZE, OBS_SIZE, K);   // K

  // x = x + K*y
  float Ky[STATE_SIZE] = {0};
  matMul(K, STATE_SIZE, OBS_SIZE, y, OBS_SIZE, 1, Ky);
  for (int i = 0; i < STATE_SIZE; i++) {
    x[i] += Ky[i];
  }

  // P = (I - K*H)*P
  float KH[STATE_SIZE * STATE_SIZE] = {0};
  matMul(K, STATE_SIZE, OBS_SIZE, H, OBS_SIZE, STATE_SIZE, KH);  // K*H
  float I[STATE_SIZE * STATE_SIZE] = {0};
  matIdentity(I, STATE_SIZE);
  matSub(I, KH, STATE_SIZE, STATE_SIZE, tmp1);                            // I-KH
  matMul(tmp1, STATE_SIZE, STATE_SIZE, P, STATE_SIZE, STATE_SIZE, tmp2);  // (I-KH)*P
  matCopy(tmp2, STATE_SIZE, STATE_SIZE, P);
}

// 获取当前状态
float * kalman_get_state(void) { return x; }

// ==================== 设置模型参数 ====================
// 主函数中调用此函数，设置F,Q,H,R
// 在init前直接对全局 F,Q,H,R 赋值
