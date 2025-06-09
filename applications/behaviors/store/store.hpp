#ifndef STORE_HPP
#define STORE_HPP

#include "behaviors/behavior_status.hpp"
constexpr float x_stretched = 0.5;   // m
constexpr float z_positionA = 0.7;   // m
constexpr float z_positionB = 0.4;   // m
constexpr float z_positionC = 0.6;   // m
constexpr float z_positionD = 0.8;   // m
constexpr float z_positionE = 0.72;  // m
BehaviorStatus store(bool layer);

#endif  // STORE_HPP