#pragma once

#include <Eigen/Dense>

namespace coords {

using Coords = Eigen::Vector3d;

// Euclidean distance between two 3D points
inline double distanceTo(const Coords &a, const Coords &b) {
    return (a - b).norm();
}

// Euclidean norm of a 3D vector
inline double norm(const Coords &v) {
    return v.norm();
}

} // namespace coords
