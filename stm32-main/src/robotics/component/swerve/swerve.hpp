#pragma once

#include <memory>

#include "motor.hpp"

#include "../../filter/pid.hpp"
#include "../../filter/angle_smoother.hpp"
#include "../../filter/angled_motor.hpp"
#include "../../sensor/gyro/base.hpp"
#include "../../node/node.hpp"
#include "../../types/angle_joystick_2d.hpp"
#include "../../types/joystick_2d.hpp"
#include "../../types/vector.hpp"
#include "../../types/angle.hpp"

namespace robotics::component {
namespace swerve {
class Swerve {
 public:
  struct Config {
    float angle_offsets[3];
  };

  Node<float> robot_angle;  // in: feedback robot angle

  Node<types::JoyStick2D> move_ctrl;  // in: move velocity
  Node<float> angle_ctrl;             // in: angle rotation

  std::array<Motor*, 3> motors;  // for injection

  filter::PID<float> angle{0.7f, 0.30f, 0.15f};  // robot angle pid
 private:
  filter::AngleNormalizer<float> rot_in_normalizer;  // ctrl angle normalizer
  filter::AngleNormalizer<float>
      self_rot_y_normalizer;  // robot angle normalizer

 public:
  Swerve(Config& config)
      : motors{
            new Motor(config.angle_offsets[0]),
            new Motor(config.angle_offsets[1]),
            new Motor(config.angle_offsets[2]),
        } {
    // robot_angle >> normalizer >> anglePID
    robot_angle.Link(self_rot_y_normalizer.input);
    self_rot_y_normalizer.output.Link(angle.fb_);

    // angle_ctrl >> normalizer >> anglePID
    angle_ctrl.Link(rot_in_normalizer.input);
    rot_in_normalizer.output.Link(angle.goal_);

    for (size_t i = 0; i < 3; i++) {
      // move_ctrl >> motor
      move_ctrl.Link(motors[i]->velocity);

      // anglePID >> motor
      angle.output_.Link(motors[i]->rotation);
    }
  }

  void Update(float dt) {
    rot_in_normalizer.Update();
    self_rot_y_normalizer.Update();
    angle.Update(dt);
  }
};
}  // namespace swerve

using Swerve = swerve::Swerve;
}  // namespace robotics::component