#pragma once

#include <Eigen/Dense>

namespace coords {

using Coords = Eigen::Vector3f;

// Euclidean distance between two 3D points
inline float distanceTo(const Coords &a, const Coords &b) {
    return (a - b).norm();
}

// Euclidean norm of a 3D vector
inline float norm(const Coords &v) {
    return v.norm();
}

} // namespace coords
