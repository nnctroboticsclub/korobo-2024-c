#pragma once

#include <ikakoMDC.h>
#include "motor.hpp"

namespace robotics::node {
class ikakoMDCMotor : public Motor<float> {
  ikakoMDC *mdc_ = nullptr;

  void SetSpeed(float speed) override {
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
}  // namespace robotics::node