#pragma once

#include <memory>

#include "motor.hpp"

#include "robotics/filter/pid.hpp"
#include "robotics/filter/angle_smoother.hpp"
#include "robotics/fusion/angled_motor.hpp"
#include "robotics/sensor/gyro/base.hpp"
#include "vector.hpp"
#include <ikakoMDC.h>

namespace robotics::component::swerve {

class Steering {
 public:
  std::array<Motor, 3> motors;
  std::shared_ptr<sensor::gyro::Base> gyro;
  filter::PID<float> angle_pid{0.7f, 0.30f, 0.15f};

 private:
  filter::AngleNormalizer<float> rot_in_normalizer;
  filter::AngleNormalizer<float> self_rot_y_normalizer;

 private:
  void MoveAndRotate(Vector<float, 2> velocity, float rotation_in_raw,
                     float angle_power, float dt) {
    // Rotation PID
    float rotation_in = rot_in_normalizer.Update(rotation_in_raw);
    float rotation_fb =
        self_rot_y_normalizer.Update(gyro->GetHorizontalOrientation());
    float rotation = toggle.isOn
                         ? rotation_in
                         : angle_pid.Update(rotation_in, rotation_fb, dt);

    for (auto& motor : motors) {
      motor.SetDrivePower(velocity, rotation);
    }
  }
};
}  // namespace robotics::component::swerve