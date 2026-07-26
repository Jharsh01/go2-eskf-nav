// slip_infer.cpp
// Deterministic inference driver for cross-validating the C++ SlipModel against
// the NumPy twin (scripts/slip_reference.py), mirroring tools/replay_eskf.cpp.
//
//   slip_infer <weights.txt> <features.csv> <output.csv>
//
// features.csv : header line, then one raw feature vector per row
//                (slip_feat::DIM comma-separated values — see slip_model.hpp).
// output.csv   : one slip score per input row.

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "go2_eskf/slip_model.hpp"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: slip_infer <weights.txt> <features.csv> <output.csv>\n";
    return 1;
  }
  go2_eskf::SlipModel model;
  try {
    model = go2_eskf::SlipModel::loadFromFile(argv[1]);
  } catch (const std::exception& e) {
    std::cerr << "failed to load weights: " << e.what() << "\n";
    return 1;
  }

  std::ifstream in(argv[2]);
  std::ofstream out(argv[3]);
  if (!in || !out) {
    std::cerr << "could not open features/output file\n";
    return 1;
  }

  out << std::setprecision(17);
  std::string line;
  std::getline(in, line);  // skip header
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::vector<double> vals;
    std::string cell;
    while (std::getline(ss, cell, ',')) vals.push_back(std::stod(cell));
    if (static_cast<int>(vals.size()) != model.inputDim()) {
      std::cerr << "feature dim mismatch: got " << vals.size() << " expected "
                << model.inputDim() << "\n";
      return 1;
    }
    Eigen::VectorXd f(model.inputDim());
    for (int i = 0; i < model.inputDim(); ++i) f(i) = vals[i];
    out << model.predict(f) << "\n";
  }
  return 0;
}
