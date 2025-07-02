#ifndef SP__SWERVE__OMNI_HPP
#define SP__SWERVE__OMNI_HPP

namespace sp
{
class Swerve_Omni
{
public:
  // swerve_wheel_radius: 舵轮轮子半径, 单位: m
  // omni_wheel_radius: 全向轮轮子半径, 单位: m
  // half_length: 前后轮距离的一半, 单位: m
  // half_width: 左右轮距离的一半, 单位: m
  // swerve_invert_pivot: 舵向电机输出轴朝内
  // omni_invert_pivot: 全向电机输出轴朝内
  // k：底盘中心到轮子中心的距离, 单位: m
  Swerve_Omni(
    float swerve_wheel_radius, float omni_wheel_radius, float half_length, float half_width,
    bool swerve_invert_pivot = true, bool omni_invert_pivot = true, float k = -1.0f);

  float speed_lf;  // 只读! calc()计算结果, 左前轮转速, 单位: rad/s
  float speed_lr;  // 只读! calc()计算结果, 左后轮转速, 单位: rad/s
  float speed_rf;  // 只读! calc()计算结果, 右前轮转速, 单位: rad/s
  float speed_rr;  // 只读! calc()计算结果, 右后轮转速, 单位: rad/s

  float angle_lr;  // 只读! calc()计算结果, 左后舵角度, 单位: rad
  float angle_rf;  // 只读! calc()计算结果, 右前舵角度, 单位: rad

  float vx;        // 只读! update()计算结果, 底盘x方向速度, 单位: m/s
  float vy;        // 只读! update()计算结果, 底盘y方向速度, 单位: m/s
  float wz;        // 只读! update()计算结果, 底盘z方向角速度, 单位: rad/s
  float odom_x;    // 只读! update()计算结果, 底盘中心在x方向位置, 单位: m
  float odom_y;    // 只读! update()计算结果, 底盘中心在y方向位置, 单位: m
  float odom_yaw;  // 只读！update()计算结果, 底盘yaw角度, 单位: rad

  // yaw_lf/lr/rf/rr: 各舵电机的初始角度, 注意此时各轮电机输出轴朝向要相同, 而不是相对! 单位: rad
  void init(float yaw_lr, float yaw_rf);

  // 底盘速度 -> 各轮转速和各舵角度
  // vx: 前进速度, 单位: m/s
  // vy: 左移速度, 单位: m/s
  // wz: 单位: rad/s, 正方向: 大拇指朝上右手螺旋方向
  // yaw_lf/lr/rf/rr: 各舵当前角度, 单位: rad
  void calc(float vx, float vy, float wz, float yaw_lr, float yaw_rf);

  void update(float speed_lf, float speed_rr, float dt);

private:
  const float swerve_r_;
  const float omni_r_;
  const float l_;
  const float w_;
  const float k_;

  const float swerve_sign_;
  const float omni_sign_;

  float yaw_offset_lr_;
  float yaw_offset_rf_;

  // 各轮速度向量 -> 各轮转速和各舵角度
  void convert(const float v[2], float yaw, float yaw_offset, float & angle, float & speed);
};

}  // namespace sp

#endif  // SP__SWERVE__OMNI_HPP