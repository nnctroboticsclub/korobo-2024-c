#pragma once

#include <memory>

#include "robotics/filter/pid.hpp"
#include "robotics/filter/angle_smoother.hpp"
#include "robotics/fusion/angled_motor.hpp"
#include "robotics/sensor/gyro/base.hpp"
#include "vector.hpp"
#include <ikakoMDC.h>

namespace robotics::comopnent {

using robotics::filter::PID;

class SteeringMotor {
  std::shared_ptr<output::Motor<float>> drive_;
  fusion::AngledMotor<float> steer_;

  Vector<float, 2> normal_vector_;

 public:
  SteeringMotor(std::shared_ptr<output::Motor<float>> drive,
                fusion::AngledMotor<float> steer, float angle_deg)
      : drive_(drive), steer_(steer) {
    normal_vector_ = Vector<float, 2>(cos(angle_deg * M_PI / 180),
                                      sin(angle_deg * M_PI / 180));
  }

  void SetPowerRaw(Vector<float, 2> vector) {
    float magnitude = vector.Magnitude();
    float angle = atan2(vector[1], vector[0]) * 180 / M_PI;

    steer_.SetTargetAngle(angle);
    drive_->SetSpeed(magnitude);
  }

  /**
   * @brief Set Driving Power of the Swerve Drive
   *
   * @param velocity Velocity Vector
   * @param rotation Rotation Power (0.0 ~ 360.0)
   */
  void SetDrivePower(Vector<float, 2> velocity, float rotation) {
    auto vector = velocity + normal_vector_ * rotation / 90;
    SetPowerRaw(vector);
  }

  void Update(float dt) { steer_.Update(dt); }
};

class Steering {
 public:
  std::shared_ptr<sensor::gyro::Base> gyro;

  std::array<SteeringMotor, 3> motors;
  filter::AngleNormalizer<float> rot_in_normalizer;
  filter::AngleNormalizer<float> self_rot_y_normalizer;

 private:
  PID<float> angle_pid{0.7f, 0.30f, 0.15f};

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
}  // namespace robotics::comopnent