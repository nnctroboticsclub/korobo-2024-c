#pragma once

#include <mbed.h>
#include "motor.hpp"

namespace robotics::node {
class BLDC : public Motor<float> {
  bool initialized = false;
  PwmOut pwmout_;
  int min_pulsewidth_, max_pulsewidth_;

  void SetSpeed(float speed) override {
    if (speed < 0) {
      // ESC ussualy doesn't support negative speed.
      pwmout_.pulsewidth_us(min_pulsewidth_);
    }

    pwmout_.pulsewidth_us(min_pulsewidth_ +
                          (max_pulsewidth_ - min_pulsewidth_) * speed);
  }

 public:
  BLDC(PinName pin, int min_pulsewidth, int max_pulsewidth)
      : pwmout_(pin),
        min_pulsewidth_(min_pulsewidth),
        max_pulsewidth_(max_pulsewidth) {
    pwmout_.period_us(2000);
    pwmout_.pulsewidth_us(0);
  }

  void Init() {
    if (initialized) return;
    initialized = true;

    pwmout_.pulsewidth_us(max_pulsewidth_);
    ThisThread::sleep_for(1s);

    pwmout_.pulsewidth_us(min_pulsewidth_);
    ThisThread::sleep_for(1s);
  }
};
}  // namespace robotics::node