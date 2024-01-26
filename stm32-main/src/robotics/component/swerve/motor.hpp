#pragma once

#include <math.h>
#include <memory>

#include "../../types/vector.hpp"
#include "../../filter/angled_motor.hpp"
#include "../../filter/joystick2angle.hpp"
#include "../../node/motor.hpp"

namespace robotics::component::swerve {

class Motor {
 public:
  Node<Vector<float, 2>> velocity;  // in: velocity
  Node<float> rotation;             // in: rotation angle

  Node<float> drive_;                 // out: drive power
  filter::AngledMotor<float> steer_;  // out: steer angle

 private:
  Vector<float, 2> normal_vector_;
  filter::Joystick2Angle angle_power;

  void UpdateAnglePower() {
    auto rot = rotation.GetValue();

    if (rot > 180) rot -= 360;

    auto vector = velocity.GetValue() + normal_vector_ * rot / 90;
    angle_power.in.SetValue(vector);
  }

 public:
  Motor(float angle_deg) : drive_(0) {
    normal_vector_[0] = std::cos(angle_deg * M_PI / 180);
    normal_vector_[1] = std::sin(angle_deg * M_PI / 180);

    velocity.SetChangeCallback(
        [this](Vector<float, 2> vector) { UpdateAnglePower(); });

    rotation.SetChangeCallback([this](float angle) { UpdateAnglePower(); });

    angle_power.out.SetChangeCallback([this](types::AngleStick2D angle_vector) {
      steer_.goal.SetValue(angle_vector.angle);
      drive_.SetValue(angle_vector.magnitude);
    });
  }

  Motor(Motor const&) = delete;
  Motor& operator=(Motor const&) = delete;

  Motor(Motor&&) = default;
  Motor& operator=(Motor&&) = default;
};

}  // namespace robotics::component::swerve