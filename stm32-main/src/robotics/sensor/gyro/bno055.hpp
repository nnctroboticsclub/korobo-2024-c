#pragma once

#include <bno055.h>
#include <rtos.h>

#include "base.hpp"
#include "../../types/vector.hpp"

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace robotics::sensor::gyro {
class BNO055 : public Base {
  ::BNO055 bno055_;
  rtos::Thread thread;
  rtos::Thread monitor_thread;

  Timer timer;

  float horizontal_orientation_ = -1;

  void ThreadMain() {
    if (!bno055_.check()) {
      printf("BNO055 not detected\n");
      return;
    }

    bno055_.reset();

    bno055_.setpowermode(POWER_MODE_NORMAL);
    bno055_.setmode(OPERATION_MODE_NDOF);

    bno055_.set_angle_units(DEGREES);
    bno055_.set_accel_units(MPERSPERS);

    timer.start();
    while (true) {
      bno055_.get_angles();
      bno055_.get_lia();

      horizontal_orientation_ = bno055_.euler.yaw;
    }
  }

  void Monitor() {
    while (true) {
      printf("Yaw: %+6.2f\n", horizontal_orientation_);
      rtos::ThisThread::sleep_for(50ms);
    }
  }

 public:
  BNO055(PinName SDA, PinName SDL) : bno055_(SDA, SDL) {}

  void Init(bool monitor = false) {
    thread.start(callback(this, &BNO055::ThreadMain));
    if (monitor) {
      monitor_thread.start(callback(this, &BNO055::Monitor));
    }
  }

  float GetHorizontalOrientation() { return horizontal_orientation_; }
};
}  // namespace robotics::sensor::gyro