#pragma once

#include <cmath>

#include "../types/angle_joystick_2d.hpp"
#include "../types/joystick_2d.hpp"

#include "../node/node.hpp"

namespace robotics::filter {
class Joystick2Angle {
 public:
  Node<JoyStick2D> in;
  Node<AngleStick2D> out;

  Joystick2Angle() {
    in.SetChangeCallback([this](JoyStick2D vector) {
      AngleStick2D angle_vector;

      angle_vector.magnitude = vector.Magnitude();
      angle_vector.angle = std::atan2(vector[1], vector[0]) * 180 / M_PI;

      out.SetValue(angle_vector);
    });
  }
};
}  // namespace robotics::filter