#pragma once

namespace robotics::sensor::gyro {
class Base {
 public:
  virtual float GetHorizontalOrientation() = 0;
};
}  // namespace robotics::sensor