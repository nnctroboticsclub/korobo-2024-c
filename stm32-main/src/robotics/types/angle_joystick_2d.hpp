#pragma once
namespace robotics {
inline namespace types {

class AngleStick2D {
 public:
  float magnitude;
  float angle;  // clockwise

  AngleStick2D() : magnitude(0), angle(0) {}
  AngleStick2D(float magnitude, float angle)
      : magnitude(magnitude), angle(angle) {}

  bool operator==(AngleStick2D const& other) const {
    return magnitude == other.magnitude && angle == other.angle;
  }
  bool operator!=(AngleStick2D const& other) const { return !(*this == other); }
};

}  // namespace types
}  // namespace robotics