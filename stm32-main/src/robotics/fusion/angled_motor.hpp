#pragma once

#include "../filter/pid.hpp"
#include "../output/motor.hpp"
#include "../sensor/encoder/base.hpp"

namespace robotics::fusion {
template <typename T>
class AngledMotor {
 public:
  struct Config {
    std::unique_ptr<sensor::encoder::Absolute<T>> encoder;
    std::unique_ptr<output::Motor<T>> motor;
  };

  filter::PID<float> angle_;
  float target_angle_;
  std::unique_ptr<sensor::encoder::Absolute<T>> encoder_;
  std::unique_ptr<output::Motor<T>> motor_;

 public:
  AngledMotor(Config&& config) {
    encoder_ = std::move(config.encoder);
    motor_ = std::move(config.motor);
  }

  input::IInputController<PIDGains>* GetPIDController() {
    return angle_.GetController();
  }

  void SetTargetAngle(float angle) { target_angle_ = angle; }

  void Update(float dt) {
    float feedback = encoder_->GetAngle();
    float output = angle_.Update(target_angle_, feedback, dt);
    motor_->SetSpeed(output);
  }
};
}  // namespace robotics::fusion