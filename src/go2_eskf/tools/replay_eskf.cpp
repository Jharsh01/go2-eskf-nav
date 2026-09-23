// replay_eskf.cpp
// Deterministic replay driver for cross-validation against the NumPy twin.
//
//   replay_eskf <input.csv> <output.csv>
//
// input.csv  rows: type,dt,a0,a1,a2,gz,roll,pitch,m0,m1   (header on line 1)
//   type 0 = IMU predict   uses dt,a0..a2,gz,roll,pitch
//   type 1 = leg odometry  uses m0,m1 (body vx,vy)
//   type 2 = GPS           uses m0,m1 (world x,y)
//   type 3 = absolute yaw  uses m0 (heading, rad, ENU) — magnetometer
// output.csv rows: the 8 nominal-state values after each processed event.
//
// The measurement covariances below MUST match scripts/eskf_reference.py.

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "go2_eskf/eskf_core.hpp"

namespace {
const Eigen::Matrix2d kLegR = (Eigen::Matrix2d() << 0.04, 0, 0, 0.04).finished();
const Eigen::Matrix2d kGpsR = (Eigen::Matrix2d() << 0.25, 0, 0, 0.25).finished();
constexpr double kYawR = 0.0225;  // (0.15 rad)^2 — matches YAW_R in the twin
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: replay_eskf <input.csv> <output.csv>\n";
    return 1;
  }
  std::ifstream in(argv[1]);
  std::ofstream out(argv[2]);
  if (!in || !out) {
    std::cerr << "could not open input/output file\n";
    return 1;
  }

  go2_eskf::EskfCore::Config cfg;  // all defaults — matches the NumPy twin
  go2_eskf::EskfCore eskf(cfg);

  out << std::setprecision(17);
  std::string line;
  std::getline(in, line);  // skip header
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::vector<double> f;
    std::string cell;
    while (std::getline(ss, cell, ',')) f.push_back(std::stod(cell));
    // type,dt,a0,a1,a2,gz,roll,pitch,m0,m1
    const int type = static_cast<int>(f[0]);
    if (type == 0) {
      eskf.predictImu(Eigen::Vector3d(f[2], f[3], f[4]), f[5], f[6], f[7], f[1]);
    } else if (type == 1) {
      eskf.correctLegOdom(Eigen::Vector2d(f[8], f[9]), kLegR);
    } else if (type == 2) {
      eskf.correctGps(Eigen::Vector2d(f[8], f[9]), kGpsR);
    } else if (type == 3) {
      eskf.correctYaw(f[8], kYawR);
    }
    const auto& x = eskf.state();
    for (int i = 0; i < go2_eskf::kStateDim; ++i) {
      out << x(i) << (i + 1 < go2_eskf::kStateDim ? "," : "\n");
    }
  }
  return 0;
}
