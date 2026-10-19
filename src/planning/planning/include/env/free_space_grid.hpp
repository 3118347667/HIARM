#pragma once

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

namespace env {

// Discretization for the existing path search, without an occupancy map.
// Every finite position is treated as free space; this does not provide avoidance.
struct FreeSpaceGrid {
  explicit FreeSpaceGrid(double grid_resolution = 0.1)
      : resolution(grid_resolution) {
    if (!std::isfinite(resolution) || resolution <= 0.0) {
      throw std::invalid_argument("grid_resolution must be positive");
    }
  }

  double resolution;

  Eigen::Vector3i pos2idx(const Eigen::Vector3d &p) const {
    return (p / resolution).array().floor().cast<int>();
  }

  Eigen::Vector3d idx2pos(const Eigen::Vector3i &idx) const {
    return (idx.cast<double>().array() + 0.5).matrix() * resolution;
  }

  bool isOccupied(const Eigen::Vector3i &) const { return false; }
  bool isOccupied(const Eigen::Vector3d &p) const { return !p.allFinite(); }
};

} // namespace env
