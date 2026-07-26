// slip_model.hpp
// Phase 3 — slip-adaptive leg-odometry covariance.
//
// A tiny, dependency-free Eigen MLP that maps per-step locomotion features to a
// slip score s in [0, 1], used to inflate the leg-odometry measurement
// covariance:
//
//     R_leg  <-  R_leg * (1 + lambda * s)^2
//
// Feet planted (s -> 0): trust leg odometry. Feet slipping (s -> 1): distrust
// it, so the filter leans on IMU + GPS instead — exactly the failure mode of
// CHAMP's stance-foot assumption.
//
// The network is trained in PyTorch (scripts/train_slip_model.py), exported as a
// plain-text weights file, and run here through a few Eigen matmuls. The NumPy
// twin (scripts/slip_reference.py) mirrors this forward pass line-for-line and
// is cross-validated against it (scripts/cross_validate_slip.py), exactly like
// EskfCore <-> eskf_reference.py.
//
// Header-only and ROS-free so it unit-tests in isolation and links into both
// the node and the offline tools.

#pragma once

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace go2_eskf {

// --- Feature vector layout -------------------------------------------------
// MUST stay in sync with SLIP_FEATURES in scripts/slip_reference.py and the
// feature order produced by SlipFeatures::toVector().
namespace slip_feat {
constexpr int CMD_MINUS_LEG_VX = 0;   // commanded - measured body vx   [m/s]
constexpr int CMD_MINUS_LEG_VY = 1;   // commanded - measured body vy   [m/s]
constexpr int CMD_MINUS_GYRO_WZ = 2;  // commanded - measured yaw rate  [rad/s]
constexpr int LEG_SPEED = 3;          // |leg-odom body velocity|       [m/s]
constexpr int JOINT_VEL_MEAN = 4;     // mean |joint velocity|          [rad/s]
constexpr int JOINT_VEL_MAX = 5;      // max  |joint velocity|          [rad/s]
constexpr int ACCEL_HORIZ = 6;        // horizontal motion-accel magn.  [m/s^2]
constexpr int CONTACT_FRAC = 7;       // fraction of feet in contact    [0..1]
constexpr int DIM = 8;
}  // namespace slip_feat

// Raw per-step quantities the node can gather from cmd_vel, leg odometry,
// joint_states and the IMU. toVector() turns them into the model's input
// feature vector. The *intuition* behind each feature: when feet slip, the
// commanded motion and joint motion persist but the measured body velocity
// (and contact) drop — so the disagreement terms spike.
struct SlipFeatures {
  double cmd_vx = 0.0, cmd_vy = 0.0, cmd_wz = 0.0;  // commanded body twist
  double leg_vx = 0.0, leg_vy = 0.0;                // measured leg-odom body vel
  double gyro_wz = 0.0;                             // measured yaw rate
  double joint_vel_mean = 0.0, joint_vel_max = 0.0; // |joint velocity| stats
  double accel_horiz = 0.0;                         // horizontal motion accel
  double contact_frac = 1.0;                        // feet-in-contact fraction

  Eigen::VectorXd toVector() const {
    Eigen::VectorXd f(slip_feat::DIM);
    f[slip_feat::CMD_MINUS_LEG_VX] = cmd_vx - leg_vx;
    f[slip_feat::CMD_MINUS_LEG_VY] = cmd_vy - leg_vy;
    f[slip_feat::CMD_MINUS_GYRO_WZ] = cmd_wz - gyro_wz;
    f[slip_feat::LEG_SPEED] = std::hypot(leg_vx, leg_vy);
    f[slip_feat::JOINT_VEL_MEAN] = joint_vel_mean;
    f[slip_feat::JOINT_VEL_MAX] = joint_vel_max;
    f[slip_feat::ACCEL_HORIZ] = accel_horiz;
    f[slip_feat::CONTACT_FRAC] = contact_frac;
    return f;
  }
};

class SlipModel {
 public:
  enum class Act { kNone, kRelu, kSigmoid };

  struct Layer {
    Eigen::MatrixXd W;  // (out x in)
    Eigen::VectorXd b;  // (out)
    Act act = Act::kNone;
  };

  SlipModel() = default;

  bool loaded() const { return !layers_.empty(); }
  int inputDim() const { return input_dim_; }

  // Forward pass on a *raw* (un-normalized) feature vector. The model applies
  // its stored input standardization (x - mean) / std first. Returns the final
  // scalar; with a trailing sigmoid layer this is a slip score in [0, 1].
  double predict(const Eigen::VectorXd& features) const {
    if (!loaded()) throw std::runtime_error("SlipModel::predict — no weights loaded");
    if (features.size() != input_dim_) {
      throw std::invalid_argument("SlipModel::predict — feature dim mismatch");
    }
    Eigen::VectorXd x = (features - mean_).cwiseQuotient(std_);
    for (const Layer& l : layers_) {
      x = l.W * x + l.b;
      applyActivation(x, l.act);
    }
    if (x.size() != 1) {
      throw std::runtime_error("SlipModel::predict — output is not scalar");
    }
    return x(0);
  }

  double predict(const SlipFeatures& feat) const { return predict(feat.toVector()); }

  // Slip -> leg covariance inflation. Bigger s => bigger R => the filter trusts
  // leg odometry less (see EskfCore::correctLegOdom). lambda scales sensitivity.
  static Eigen::Matrix2d adaptLegCovariance(const Eigen::Matrix2d& R_base,
                                            double slip, double lambda) {
    const double f = 1.0 + lambda * slip;
    return R_base * (f * f);
  }

  // Parse the plain-text weights file written by scripts/slip_reference.py and
  // scripts/train_slip_model.py. Format (lines starting with '#' are comments):
  //
  //   input_dim <D>
  //   mean  <m_0> ... <m_{D-1}>
  //   std   <s_0> ... <s_{D-1}>
  //   layers <L>
  //   layer <in> <out> <relu|sigmoid|none>
  //   <out rows of W, each <in> values>
  //   <one row of b, <out> values>
  //   ... (repeated for each layer)
  static SlipModel loadFromFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("SlipModel: cannot open weights file: " + path);
    return loadFromStream(in);
  }

  static SlipModel loadFromStream(std::istream& in) {
    SlipModel m;
    std::string tok = nextToken(in);
    if (tok != "input_dim") throw std::runtime_error("SlipModel: expected 'input_dim'");
    m.input_dim_ = static_cast<int>(readNum(in));

    if (nextToken(in) != "mean") throw std::runtime_error("SlipModel: expected 'mean'");
    m.mean_ = readVector(in, m.input_dim_);
    if (nextToken(in) != "std") throw std::runtime_error("SlipModel: expected 'std'");
    m.std_ = readVector(in, m.input_dim_);
    // Guard against divide-by-zero on constant features.
    for (int i = 0; i < m.std_.size(); ++i)
      if (std::abs(m.std_(i)) < 1e-12) m.std_(i) = 1.0;

    if (nextToken(in) != "layers") throw std::runtime_error("SlipModel: expected 'layers'");
    const int n_layers = static_cast<int>(readNum(in));

    int expected_in = m.input_dim_;
    for (int li = 0; li < n_layers; ++li) {
      if (nextToken(in) != "layer") throw std::runtime_error("SlipModel: expected 'layer'");
      const int in_dim = static_cast<int>(readNum(in));
      const int out_dim = static_cast<int>(readNum(in));
      const Act act = parseAct(nextToken(in));
      if (in_dim != expected_in) {
        throw std::runtime_error("SlipModel: layer input dim does not chain");
      }
      Layer l;
      l.act = act;
      l.W.resize(out_dim, in_dim);
      for (int r = 0; r < out_dim; ++r)
        for (int c = 0; c < in_dim; ++c) l.W(r, c) = readNum(in);
      l.b = readVector(in, out_dim);
      m.layers_.push_back(std::move(l));
      expected_in = out_dim;
    }
    if (expected_in != 1) {
      throw std::runtime_error("SlipModel: final layer must output a scalar");
    }
    return m;
  }

 private:
  static void applyActivation(Eigen::VectorXd& x, Act a) {
    switch (a) {
      case Act::kRelu:
        x = x.cwiseMax(0.0);
        break;
      case Act::kSigmoid:
        for (int i = 0; i < x.size(); ++i) x(i) = 1.0 / (1.0 + std::exp(-x(i)));
        break;
      case Act::kNone:
        break;
    }
  }

  static Act parseAct(const std::string& s) {
    if (s == "relu") return Act::kRelu;
    if (s == "sigmoid") return Act::kSigmoid;
    if (s == "none") return Act::kNone;
    throw std::runtime_error("SlipModel: unknown activation '" + s + "'");
  }

  // Token reader that skips '#' comment lines and any whitespace.
  static std::string nextToken(std::istream& in) {
    std::string t;
    while (in >> t) {
      if (!t.empty() && t[0] == '#') {
        std::string rest;
        std::getline(in, rest);  // discard the remainder of the comment line
        continue;
      }
      return t;
    }
    throw std::runtime_error("SlipModel: unexpected end of weights file");
  }

  static double readNum(std::istream& in) { return std::stod(nextToken(in)); }

  static Eigen::VectorXd readVector(std::istream& in, int n) {
    Eigen::VectorXd v(n);
    for (int i = 0; i < n; ++i) v(i) = readNum(in);
    return v;
  }

  int input_dim_ = 0;
  Eigen::VectorXd mean_, std_;
  std::vector<Layer> layers_;
};

}  // namespace go2_eskf
