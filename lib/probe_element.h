#pragma once

#include <cstddef>

struct ProbeElement {
  bool enabled = true; // dead/active element

  float weight = 1.0f; // apodization
  std::size_t delay = 0U; // in samples

  bool isActive() const noexcept { return enabled; };
};
