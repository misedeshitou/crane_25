#ifndef MOVE_HPP
#define MOVE_HPP

#include "behaviors/behavior_status.hpp"

constexpr float DIST_EPS = 0.005;      // mm
constexpr float ANGLE_EPS = 5 / 57.3;  // rad

BehaviorStatus move_x(float x);
BehaviorStatus move_z(float z);
BehaviorStatus switch_claw(bool claw);
BehaviorStatus move(float x, float z, bool claw);

#endif  // MOVE_HPP