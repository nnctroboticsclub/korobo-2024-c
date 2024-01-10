#pragma once

#include <ikakoMDC.h>
#include "motor.hpp"

namespace robotics::output {
class ikakoMDCMotor : public Motor<float>, public ikakoMDC {
  int min_speed_, max_speed_;

 public:
  ikakoMDCMotor(int motor_num, int min_speed, int max_speed)
      : ikakoMDC(                  //
            motor_num,             //
            min_speed, max_speed,  //
            0.001, 0,              //
            2.7, 0, 0.0005,        // MDC PID
            0.01                   //
            ),
        min_speed_(min_speed),
        max_speed_(max_speed) {}

  void SetSpeed(float speed) override {
    if (speed > 0) {
      ikakoMDC::set_speed(speed * max_speed_);
    } else if (speed < 0) {
      ikakoMDC::set_speed(speed * min_speed_);
    } else {
      ikakoMDC::set_speed(0);
    }
  }
};
}  // namespace robotics::output