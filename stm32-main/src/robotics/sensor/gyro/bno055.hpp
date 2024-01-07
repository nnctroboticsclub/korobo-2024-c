#pragma once

#include <bno055.h>
#include <rtos.h>

#include "base.hpp"
#include "../../vector.hpp"

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace robotics::sensor::gyro {
class Gyro : public Base {
  BNO055 bno055_;
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

      float yaw = bno055_.euler.yaw;
      float pitch = bno055_.euler.pitch;
      float roll = bno055_.euler.roll;

      horizontal_orientation_ = yaw;
    }
  }

  void Monitor() {
    while (true) {
      printf("Yaw: %+6.2f\n", horizontal_orientation_);
      rtos::ThisThread::sleep_for(50ms);
    }
  }

 public:
  Gyro(PinName SDA, PinName SDL) : bno055_(SDA, SDL) {}

  void Init(bool monitor = false) {
    thread.start(callback(this, &Gyro::ThreadMain));
    if (monitor) {
      monitor_thread.start(callback(this, &Gyro::Monitor));
    }
  }

  float GetHorizontalOrientation() { return horizontal_orientation_; }
};
}  // namespace robotics::sensor::gyro