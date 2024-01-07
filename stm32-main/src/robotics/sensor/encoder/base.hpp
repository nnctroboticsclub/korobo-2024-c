#pragma once

namespace robotics::sensor::encoder {
template <typename T>
class Absolute {
 public:
  virtual T GetAngle() = 0;
};
}  // namespace robotics::sensor::encoder