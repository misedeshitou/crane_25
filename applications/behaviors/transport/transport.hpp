#ifndef TRANSPORT_HPP
#define TRANSPORT_HPP

#include "behaviors/behavior_status.hpp"
#include "hardwares/can/can.hpp"
#include "hardwares/chassis/chassis.hpp"
#include "hardwares/chassis/swerve_omni.hpp"

constexpr float DIST_EPS = 0.005;      // mm
constexpr float ANGLE_EPS = 5 / 57.3;  // rad

using namespace chassis;
extern sp::Swerve_Omni chassis_;
BehaviorStatus transport_x(float x);
BehaviorStatus transport_y(float y);
BehaviorStatus transport_yaw(float w);
BehaviorStatus transport(float x, float z, float w);

#endif  // TRANSPORT_HPP