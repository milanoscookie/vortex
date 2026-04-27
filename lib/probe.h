#pragma once

#include "probe_element.h"
#include "utils/coords.h"

#include <array>
#include <cassert>
#include <cstddef>

using namespace coords;

template <size_t NumX, size_t NumY> class Probe {
public:
  Probe(const Coords &center, float dx, float dy)
      : center_(center), dx_(dx), dy_(dy) {}

  const Coords &center() const noexcept { return center_; }
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

  Coords elementPosition(size_t ix, size_t iy) const noexcept {
    const float x_offset =
        (static_cast<float>(ix) - 0.5f * static_cast<float>(NumX - 1)) * dx_;
    const float y_offset =
        (static_cast<float>(iy) - 0.5f * static_cast<float>(NumY - 1)) * dy_;

    return center_ + Coords(x_offset, y_offset, 0.0f);
  }

  ProbeElement *data() noexcept { return elems_.data(); }
  const ProbeElement *data() const noexcept { return elems_.data(); }

  auto begin() noexcept { return elems_.begin(); }
  auto end() noexcept { return elems_.end(); }

  auto begin() const noexcept { return elems_.begin(); }
  auto end() const noexcept { return elems_.end(); }

private:
  static constexpr size_t flatten(size_t ix, size_t iy) noexcept {
    return ix * NumY + iy;
  }

  Coords center_;
  float dx_;
  float dy_;

  std::array<ProbeElement, NumX * NumY> elems_;
};
