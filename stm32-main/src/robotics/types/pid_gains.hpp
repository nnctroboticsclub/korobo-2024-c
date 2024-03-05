#pragma once

#include "vector.hpp"

namespace robotics {
inline namespace types {

struct PIDGains {
  float g;
  float p;
  float i;
  float d;

  PIDGains() : g(0), p(0), i(0), d(0) {}
  PIDGains(float g, float p, float i, float d) : g(g), p(p), i(i), d(d) {}

  void Normalize() {
    float max = std::max(std::abs(p), std::abs(i));
    max = std::max(max, std::abs(d));

    p /= max;
    i /= max;
    d /= max;
  }

  bool operator==(PIDGains const& other) const {
    return g == other.g && p == other.p && i == other.i && d == other.d;
  }
  bool operator!=(PIDGains const& other) const { return !(*this == other); }
};

}  // namespace types
}  // namespace robotics