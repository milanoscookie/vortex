#pragma once

#include "problem_description.h"
#include "utils/coords.h"

#include <array>
#include <cassert>
#include <cstddef>



template <size_t NumX, size_t NumY> class Probe {
public:
  using Coord = coords::Coords;
  using Vec3 = problem::Vec3;
  using PositionMatrix = Eigen::Matrix<float, 3, NumX * NumY>;
  using ScalarArray = Eigen::Array<float, NumX * NumY, 1>;

  struct ProbeElement {
    bool enabled = true;
    float weight = 1.0f;
    std::size_t delay = 0U;

    bool isActive() const noexcept { return enabled; }
  };

  struct Compiled {
    PositionMatrix element_positions_m = PositionMatrix::Zero();
    ScalarArray element_delay_s = ScalarArray::Zero();
    ScalarArray element_weight = ScalarArray::Zero();
  };

  Probe(const Coord &center, float dx, float dy)
      : center_(center), dx_(dx), dy_(dy) {}

  [[nodiscard]] const Coord &center() const noexcept { return center_; }
  float dx() const noexcept { return dx_; }
  float dy() const noexcept { return dy_; }

  static constexpr size_t numX() noexcept { return NumX; }
  static constexpr size_t numY() noexcept { return NumY; }
  static constexpr size_t size() noexcept { return NumX * NumY; }

  ProbeElement &operator()(size_t ix, size_t iy) noexcept {
    return elems_[flatten(ix, iy)];
  }

  const ProbeElement &operator()(size_t ix, size_t iy) const noexcept {
    return elems_[flatten(ix, iy)];
  }

  ProbeElement &at(size_t ix, size_t iy) {
    assert(ix < NumX && iy < NumY);
    return elems_[flatten(ix, iy)];
  }

  const ProbeElement &at(size_t ix, size_t iy) const {
    assert(ix < NumX && iy < NumY);
    return elems_[flatten(ix, iy)];
  }

  [[nodiscard]] Coord elementPosition(size_t ix, size_t iy) const noexcept {
    const float x_offset =
        (static_cast<float>(ix) - 0.5f * static_cast<float>(NumX - 1)) * dx_;
    const float y_offset =
        (static_cast<float>(iy) - 0.5f * static_cast<float>(NumY - 1)) * dy_;

    return center_ + Coord(x_offset, y_offset, 0.0f);
  }

  ProbeElement *data() noexcept { return elems_.data(); }
  const ProbeElement *data() const noexcept { return elems_.data(); }

  auto begin() noexcept { return elems_.begin(); }
  auto end() noexcept { return elems_.end(); }

  auto begin() const noexcept { return elems_.begin(); }
  auto end() const noexcept { return elems_.end(); }

  [[nodiscard]] Compiled compile(float sample_rate_hz) const {
    Compiled compiled;
    for (size_t ix = 0; ix < NumX; ++ix) {
      for (size_t iy = 0; iy < NumY; ++iy) {
        const size_t flat_index = flatten(ix, iy);
        compiled.element_positions_m.col(static_cast<Eigen::Index>(flat_index)) =
            elementPosition(ix, iy);
        compiled.element_delay_s(static_cast<Eigen::Index>(flat_index)) =
            static_cast<float>((*this)(ix, iy).delay) / sample_rate_hz;
        compiled.element_weight(static_cast<Eigen::Index>(flat_index)) =
            (*this)(ix, iy).isActive() ? (*this)(ix, iy).weight : 0.0f;
      }
    }
    return compiled;
  }

private:
  static constexpr size_t flatten(size_t ix, size_t iy) noexcept {
    return ix * NumY + iy;
  }

  Coord center_;
  float dx_;
  float dy_;
  std::array<ProbeElement, NumX * NumY> elems_;
};
