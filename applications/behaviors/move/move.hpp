#ifndef MOVE_HPP
#define MOVE_HPP

#include "behaviors/behavior_status.hpp"

constexpr float DIST_EPS = 0.005;      // mm
constexpr float ANGLE_EPS = 5 / 57.3;  // rad

BehaviorStatus move_x(float x);
BehaviorStatus move_y(float y);
BehaviorStatus move_z(float z);
BehaviorStatus move_yaw(float yaw);
BehaviorStatus move_roll1(float roll1);
BehaviorStatus move_end(float pitch, float roll2);
BehaviorStatus move(float x, float y, float z, float yaw, float roll1, float pitch, float roll2);

#endif  // MOVE_HPP