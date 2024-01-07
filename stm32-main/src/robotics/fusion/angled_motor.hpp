#pragma once

#include "../filter/pid.hpp"
#include "../output/motor.hpp"
#include "../sensor/encoder/base.hpp"

namespace robotics::fusion {
template <typename T>
class AngledMotor {
  filter::PID<float> angle_;
  float target_angle_;
  sensor::encoder::Absolute<T>& encoder_;
  output::Motor<T>& motor_;

 public:
  AngledMotor(PID<float> angle, sensor::encoder::Absolute<T>& encoder)
      : angle_(angle), encoder_(encoder) {}

  std::shared_ptr<filter::IPIDController> GetPIDController() {
    return angle_.GetController();
  }

  void SetTargetAngle(float angle) { target_angle_ = angle; }

  void Update(float dt) {
    float feedback = encoder_.GetAngle();
    float output = angle_.Update(target_angle_, feedback, dt);
    motor_.SetSpeed(output);
  }
};
}  // namespace robotics::fusion