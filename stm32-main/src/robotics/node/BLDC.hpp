#pragma once

#include <mbed.h>
#include "motor.hpp"

namespace robotics::node {
class BLDC : public Motor<float> {
  enum Status { Initialized, ESCInit0, Ready };

  PwmOut pwmout_;
  int min_pulsewidth_, max_pulsewidth_;
  Status status;

  void SetSpeed(float speed) override {
    if (status != Status::Ready) return;

    // ESC ussualy doesn't support negative speed.
    if (speed < 0) speed = 0;

    if (speed > 1) speed = 1;

    pwmout_.pulsewidth_us(min_pulsewidth_ +
                          (max_pulsewidth_ - min_pulsewidth_) * (speed * 0.25));
  }

 public:
  BLDC(PinName pin, int min_pulsewidth, int max_pulsewidth)
      : pwmout_(pin),
        min_pulsewidth_(min_pulsewidth),
        max_pulsewidth_(max_pulsewidth),
        status(Status::Initialized) {
    pwmout_.period_us(2000);
    pwmout_.pulsewidth_us(0);
  }

  void Init0() {
    if (status != Status::Initialized) return;

    pwmout_.pulsewidth_us(max_pulsewidth_);
    status = Status::ESCInit0;
  }
  void Init1() {
    if (status != Status::ESCInit0) return;

    pwmout_.pulsewidth_us(min_pulsewidth_);
    status = Status::Ready;
  }
};
}  // namespace robotics::node