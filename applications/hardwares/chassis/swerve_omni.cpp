#include "swerve_omni.hpp"

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
Swerve_Omni::Swerve_Omni(
  float swerve_wheel_radius, float omni_wheel_radius, float half_length, float half_width,
  bool swerve_invert_pivot, bool omni_invert_pivot, float k)
: swerve_r_(swerve_wheel_radius),
  omni_r_(omni_wheel_radius),
  l_(half_length),
  w_(half_width),
  swerve_sign_(swerve_invert_pivot ? -1.0f : 1.0f),
  omni_sign_(omni_invert_pivot ? -1.0f : 1.0f),
  k_(std::sqrt(half_length * half_length + half_width * half_width))
{
  this->speed_lf = 0.0f;
  this->speed_lr = 0.0f;
  this->speed_rf = 0.0f;
  this->speed_rr = 0.0f;
}

void Swerve_Omni::init(float yaw_lr, float yaw_rf)
{
  yaw_offset_lr_ = yaw_lr;
  yaw_offset_rf_ = yaw_rf;
}

void Swerve_Omni::calc(float vx, float vy, float wz, float yaw_lr, float yaw_rf)
{
  // 计算中间变量: 各轮速度向量
  float v_lr[2] = {vx - wz * w_, vy - wz * l_};
  float v_rf[2] = {vx + wz * w_, vy + wz * l_};

  // 计算各舵转角和各舵角度
  convert(v_lr, yaw_lr, yaw_offset_lr_, this->angle_lr, this->speed_lr);
  convert(v_rf, yaw_rf, yaw_offset_rf_, this->angle_rf, this->speed_rf);

  this->speed_lf = omni_sign_ * (-vx * std::sqrt(2) + vy * std::sqrt(2) + k_ * wz) / k_;
  this->speed_rr = omni_sign_ * (vx * std::sqrt(2) - vy * std::sqrt(2) + k_ * wz) / k_;
}
void Swerve_Omni::update(float speed_lf, float speed_rr, float dt)
{
  float sign_speed_lf = speed_lf / omni_sign_;
  float sign_speed_lr = speed_lr / omni_sign_;
  float sign_speed_rf = speed_rf / omni_sign_;
  float sign_speed_rr = speed_rr / omni_sign_;

  this->vx =
    (-sign_speed_lf - sign_speed_lr + sign_speed_rf + sign_speed_rr) / std::sqrt(2) / 4 * omni_r_;
  this->vy =
    (sign_speed_lf - sign_speed_lr + sign_speed_rf - sign_speed_rr) / std::sqrt(2) / 4 * omni_r_;
  this->wz = (sign_speed_lf + sign_speed_lr + sign_speed_rf + sign_speed_rr) / k_ / 4 * omni_r_;
  this->odom_x += (this->vx * cos(this->odom_yaw) - this->vy * sin(this->odom_yaw)) * dt;
  this->odom_y += (this->vx * sin(this->odom_yaw) + this->vy * cos(this->odom_yaw)) * dt;
  this->odom_yaw += this->wz * dt;
}
void Swerve_Omni::convert(
  const float v[2], float yaw, float yaw_offset, float & angle, float & speed)
{
  // ref: https://github.com/rm-controls/rm_controllers/blob/master/rm_chassis_controllers/src/swerve.cpp

  if (v[0] == 0.0f && v[1] == 0.0f) {
    angle = yaw;
    speed = 0.0f;
    return;
  }

  float v_angle = std::atan2(v[1], v[0]);
  float v_angle_flipped = limit_angle(v_angle + PI);
  float pivot_angle = swerve_sign_ * limit_angle(yaw - yaw_offset);

  float a = limit_angle(v_angle - pivot_angle);
  float b = limit_angle(v_angle_flipped - pivot_angle);
  float pivot_angle_set = (std::abs(a) < std::abs(b)) ? v_angle : v_angle_flipped;

  angle = limit_angle(swerve_sign_ * pivot_angle_set + yaw_offset);
  speed = std::sqrt(v[0] * v[0] + v[1] * v[1]) / swerve_r_ * std::cos(a);
}

}  // namespace sp