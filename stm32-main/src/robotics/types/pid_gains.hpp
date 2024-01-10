#pragma once

#include "vector.hpp"

namespace robotics {
inline namespace types {

struct PIDGains {
  float p;
  float i;
  float d;

  PIDGains() : p(0), i(0), d(0) {}
  PIDGains(float p, float i, float d) : p(p), i(i), d(d) {}

  bool operator==(PIDGains const& other) const {
    return p == other.p && i == other.i && d == other.d;
  }
  bool operator!=(PIDGains const& other) const { return !(*this == other); }
};

}  // namespace types
}  // namespace robotics