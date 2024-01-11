#pragma once

#include <mbed.h>
#include "motor.hpp"

namespace robotics::output {
class BLDC : public Motor<float> {
  int min_pulsewidth_, max_pulsewidth_;

  PwmOut pwmout_;

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
    pwmout_.period_ms(20);
    pwmout_.pulsewidth_us(0);
  }
};
}  // namespace robotics::output