#include "MPC.hpp"
#include "kalman_filter.hpp"

#define STATE_DIM 3
#define CONTROL_DIM 3

extern can::autoaim_measure autoaim_data;
extern float * kalman_get_state(void);

float A[STATE_DIM * STATE_DIM] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float B[STATE_DIM * CONTROL_DIM] = {0.01f, 0, 0, 0, 0.01f, 0, 0, 0, 0.01f};
float state_weights[STATE_DIM] = {10, 10, 5};
float control_weights[CONTROL_DIM] = {0.1, 0.1, 0.1};
float dt = 0.01f;
float u[CONTROL_DIM] = {0, 0, 0};
float x_d[STATE_DIM] = {0, 0, 0};

void mpc_set_target(const float * x_d_in)
{
  x_d[0] = x_d_in[0];
  x_d[1] = x_d_in[1];
  x_d[2] = x_d_in[2];
}

void mpc_set_weights(const float * state_weights_in, const float * control_weights_in)
{
  state_weights[0] = state_weights_in[0];
  state_weights[1] = state_weights_in[1];
  state_weights[2] = state_weights_in[2];
  control_weights[0] = control_weights_in[0];
  control_weights[1] = control_weights_in[1];
  control_weights[2] = control_weights_in[2];
}

void mpc_get_control(float * u_out)
{
  u_out[0] = u[0];
  u_out[1] = u[1];
  u_out[2] = u[2];
}

void mpc_set_system_matrices(const float * A_in, const float * B_in)
{
  for (int i = 0; i < STATE_DIM * STATE_DIM; i++) {
    A[i] = A_in[i];
  }
  for (int i = 0; i < STATE_DIM * CONTROL_DIM; i++) {
    B[i] = B_in[i];
  }
}

void mpc_set_time_step(float dt_in) { dt = dt_in; }

void mpc_solve_step(float * out_u)
{
  const float * x = kalman_get_state();
  float x_next[STATE_DIM] = {0};

  for (int i = 0; i < STATE_DIM; i++) {
    for (int j = 0; j < STATE_DIM; j++) {
      x_next[i] += A[i * STATE_DIM + j] * x[j];
    }
    for (int j = 0; j < CONTROL_DIM; j++) {
      x_next[i] += B[i * CONTROL_DIM + j] * u[j];
    }
  }

  float grad[CONTROL_DIM] = {0};
  for (int j = 0; j < CONTROL_DIM; j++) {
    for (int i = 0; i < STATE_DIM; i++) {
      float swi = state_weights[i] * (x_next[i] - x_d[i]);
      grad[j] += B[i * CONTROL_DIM + j] * swi;
    }
    grad[j] += control_weights[j] * u[j];
    grad[j] *= 2.0f;
  }

  float alpha = 0.1f;
  for (int i = 0; i < CONTROL_DIM; i++) {
    u[i] -= alpha * grad[i];
    if (u[i] > 1.0f) u[i] = 1.0f;
    if (u[i] < -1.0f) u[i] = -1.0f;
  }

  out_u[0] = u[0];
  out_u[1] = u[1];
  out_u[2] = u[2];
}