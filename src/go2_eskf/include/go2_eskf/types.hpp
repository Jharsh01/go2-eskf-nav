// types.hpp
// Shared types and constants for the 8-state error-state EKF (ESKF).
//
// State vector (nominal):  x = [ px py pz  vx vy vz  psi  b_g ]^T
//   px,py,pz  : robot position in the world (ENU) frame              [m]
//   vx,vy,vz  : robot velocity in the world frame                   [m/s]
//   psi       : yaw / heading                                       [rad]
//   b_g       : yaw-rate gyro bias                                  [rad/s]
//
// Roll and pitch are NOT estimated here — they are taken directly from the
// IMU (gravity makes them observable), which is exactly why the state is 8
// and not 15. See docs/DESIGN.md for the full rationale.
//
// Pure C++/Eigen — no ROS dependency, so the math can be unit-tested and
// cross-validated against the NumPy reference in isolation.

#pragma once

#include <Eigen/Dense>

namespace go2_eskf {

constexpr int kStateDim = 8;

using Vec8 = Eigen::Matrix<double, kStateDim, 1>;
using Mat8 = Eigen::Matrix<double, kStateDim, kStateDim>;

// Standard gravity. Single source of truth shared by C++ and the NumPy twin.
constexpr double kGravity = 9.80665;

// Named indices into the state vector — use these everywhere instead of magic
// numbers so the code reads like the math.
namespace idx {
constexpr int PX = 0;
constexpr int PY = 1;
constexpr int PZ = 2;
constexpr int VX = 3;
constexpr int VY = 4;
constexpr int VZ = 5;
constexpr int PSI = 6;
constexpr int BG = 7;
}  // namespace idx

}  // namespace go2_eskf
