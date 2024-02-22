#pragma once

#include <ikakoMDC.h>
#include "motor_with_encoder.hpp"

namespace robotics::node {

template <typename T = float>
class ikakoMDCMotor : public Motor<T> {
  ikakoMDC *mdc_ = nullptr;

  void SetSpeed(T speed) override {
    if (!mdc_) return;
    if (speed > 0) {
      mdc_->set_speed(speed * mdc_->max_speed);
    } else if (speed < 0) {
      mdc_->set_speed(speed * mdc_->min_speed);
    } else {
      mdc_->set_speed(0);
    }
  }

 public:
  ikakoMDCMotor(ikakoMDC &mdc) : mdc_(&mdc) {}
};

template <typename T = float>
class ikakoMDCEncoder : public Node<T> {
  ikakoMDC *mdc_ = nullptr;

 public:
  ikakoMDCEncoder(ikakoMDC &mdc) : mdc_(&mdc) {}

  void UpdateNode() {
    if (!mdc_) return;
    this->SetValue((float)mdc_->get_enc());
  }
};

}  // namespace robotics::node
namespace robotics::assembly {

template <typename T>
class ikakoMDCPair : public MotorWithEncoder<T> {
  node::ikakoMDCMotor<T> *motor_;
  node::ikakoMDCEncoder<T> *encoder_;

 public:
  ikakoMDCPair(::ikakoMDC &mdc)
      : motor_(new node::ikakoMDCMotor<T>(mdc)),
        encoder_(new node::ikakoMDCEncoder<T>(mdc)) {}

  void Update() { encoder_->UpdateNode(); }

  Node<T> &GetEncoder() override { return *encoder_; }

  node::Motor<T> &GetMotor() override { return *motor_; }
};
}  // namespace robotics::assembly