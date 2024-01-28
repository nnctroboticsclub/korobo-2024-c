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

  Node<Vector<float, 2>> vector;  // debug: vector

 private:
  Vector<float, 2> normal_vector_;
  filter::Joystick2Angle angle_power;

  void UpdateAnglePower() {
    auto rot = rotation.GetValue();
    auto vel = velocity.GetValue();

    auto vector_ = vel + normal_vector_ * rot / 90;
    vector.SetValue(vector_);
  }

 public:
  Motor(float angle_deg) : drive_(0) {
    normal_vector_[0] = std::cos(angle_deg * M_PI / 180);
    normal_vector_[1] = std::sin(angle_deg * M_PI / 180);

    velocity.SetChangeCallback(
        [this](Vector<float, 2> vector) { UpdateAnglePower(); });

    rotation.SetChangeCallback([this](float angle) { UpdateAnglePower(); });

    vector.Link(angle_power.in);

    angle_power.out.SetChangeCallback([this](types::AngleStick2D angle_vector) {
      steer_.goal.SetValue(angle_vector.angle);
      drive_.SetValue(angle_vector.magnitude);
    });
  }

  Motor(Motor const&) = delete;
  Motor& operator=(Motor const&) = delete;

  Motor(Motor&&) = default;
  Motor& operator=(Motor&&) = default;

  void Update(float dt) { this->steer_.Update(dt); }
};

}  // namespace robotics::component::swerve