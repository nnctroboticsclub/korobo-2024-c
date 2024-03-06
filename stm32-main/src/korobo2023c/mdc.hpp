#pragma once

#include <array>

#include <ikarashiCAN_mk2.h>

#include "../robotics/assembly/ikakoMDC.hpp"
#include "../robotics/assembly/dummy_motor_with_encoder.hpp"

class MDC {
  ::ikakoMDC motors_[4];
  ikakoMDC_sender sender_;
  std::array<robotics::assembly::ikakoMDCPair<float>, 4> motor_nodes_;
  ikarashiCAN_mk2 *linked_ican_;

  robotics::assembly::DummyMotorWithEncoder<float> d;

 public:
  MDC(ikarashiCAN_mk2 *can, int mdc_id)
      : motors_{ikakoMDC(4 * (mdc_id - 1) + 1, -50, 50, 0.001, 0.0, 2.7, 0,
                         0.000015, 0.01),
                ikakoMDC(4 * (mdc_id - 1) + 2, -50, 50, 0.001, 0.0, 2.7, 0,
                         0.000015, 0.01),
                ikakoMDC(4 * (mdc_id - 1) + 3, -50, 50, 0.001, 0.0, 2.7, 0,
                         0.000015, 0.01),
                ikakoMDC(4 * (mdc_id - 1) + 4, -50, 50, 0.001, 0.0, 2.7, 0,
                         0.000015, 0.01)},
        sender_(motors_, 4, can, mdc_id),
        motor_nodes_{motors_[0], motors_[1], motors_[2], motors_[3]},
        linked_ican_(can) {}

  void Tick() {
    if (sender_.read_enc() && linked_ican_->get_read_flag()) {
      for (size_t i = 0; i < 4; i++) {
        motor_nodes_[i].Update();
      }
    }
  }

  int Send() {
    auto ret = sender_.send();
    return ret;
  }

  robotics::assembly::MotorWithEncoder<float> &GetNode(int index) {
    return motor_nodes_[index];
  }
};