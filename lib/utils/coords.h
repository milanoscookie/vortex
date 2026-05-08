#pragma once

#include "problem_description.h"

#include <Eigen/Dense>

namespace coords {

using Coords = Eigen::Vector3f;

// Euclidean distance between two 3D points
inline problem::Real distanceTo(const Coords &a, const Coords &b) {
    return (a - b).norm();
}

// Euclidean norm of a 3D vector
inline problem::Real norm(const Coords &v) {
    return v.norm();
}

} // namespace coords
