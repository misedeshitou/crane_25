#ifndef TRAJECTORY_PREDICTION_HPP
#define TRAJECTORY_PREDICTION_HPP

#include <cmsis_os.h>

#include "MPC.hpp"
#include "hardwares/chassis/chassis.hpp"
#include "kalman_filter.hpp"
#include "swerve_omni.hpp"

extern float ins_gyro[3];
extern sp::Swerve_Omni chassis_;

#endif  // TRAJECTORY_PREDICTION_HPP