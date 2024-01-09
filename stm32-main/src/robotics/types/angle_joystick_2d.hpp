#pragma once
namespace robotics::types {

class AngleStick2D {
 public:
  uint8_t magnitude;
  uint8_t angle;  // clockwise

  AngleStick2D() : magnitude(0), angle(0) {}
  AngleStick2D(uint8_t magnitude, uint8_t angle)
      : magnitude(magnitude), angle(angle) {}

  bool operator==(AngleStick2D const& other) const {
    return magnitude == other.magnitude && angle == other.angle;
  }
  bool operator!=(AngleStick2D const& other) const { return !(*this == other); }
};

}  // namespace robotics::types