#pragma once

#include <array>
#include <algorithm>

#include <ikarashiCAN_mk2.h>

#include "../robotics/assembly/ikakoMDC.hpp"
#include "../robotics/assembly/dummy_motor_with_encoder.hpp"

#include "../../dcan.hpp"

class MDC {
  ::ikakoMDC motors_[4];
  ikakoMDC_sender sender_;
  std::array<robotics::assembly::ikakoMDCPair<float>, 4> motor_nodes_;
  ikarashiCAN_mk2 *linked_ican_;

  robotics::assembly::DummyMotorWithEncoder<float> d;

  int report_counter = 0;

  void ReportSpeed(DistributedCAN &can, uint8_t id) {
    std::vector<uint8_t> report(5);
    report.reserve(5);

    report[0] = 0x30 | id;
    report[1] =
        std::max(std::min(motor_nodes_[0].GetMotor().GetSpeed(), 1.0f), -1.0f) *
            127 +
        128;
    report[2] =
        std::max(std::min(motor_nodes_[1].GetMotor().GetSpeed(), 1.0f), -1.0f) *
            127 +
        128;
    report[3] =
        std::max(std::min(motor_nodes_[2].GetMotor().GetSpeed(), 1.0f), -1.0f) *
            127 +
        128;
    report[4] =
        std::max(std::min(motor_nodes_[3].GetMotor().GetSpeed(), 1.0f), -1.0f) *
            127 +
        128;

    auto ret = can.Send(0xa0, report);
    if (ret != 1) {
      printf("MDC Report: Sending the report is failed. (id: %d)\n", id);
    }
  }

  void ReportEncoder(DistributedCAN &can, uint8_t id) {
    std::vector<uint8_t> report(5);
    report.reserve(5);

    report[0] = 0x50 | id;
    report[1] = std::max(
        std::min(motor_nodes_[0].GetEncoder().GetValue() / 360.0f, 255.0f),
        0.0f);
    report[2] = std::max(
        std::min(motor_nodes_[1].GetEncoder().GetValue() / 360.0f, 255.0f),
        0.0f);
    report[3] = std::max(
        std::min(motor_nodes_[2].GetEncoder().GetValue() / 360.0f, 255.0f),
        0.0f);
    report[4] = std::max(
        std::min(motor_nodes_[3].GetEncoder().GetValue() / 360.0f, 255.0f),
        0.0f);

    auto ret = can.Send(0xa0, report);
    if (ret != 1) {
      printf("MDC Report: Sending the encoder report is failed. (id: %d)\n",
             id);
    }
  }

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

  void ReportTo(DistributedCAN &can, uint8_t id) {
    switch (report_counter) {
      case 0:
        ReportSpeed(can, id);
        break;
      case 1:
        ReportEncoder(can, id);
        break;
    }
    report_counter = (report_counter + 1) % 2;
  }

  int Send() {
    auto ret = sender_.send();
    return ret;
  }

  robotics::assembly::MotorWithEncoder<float> &GetNode(int index) {
    return motor_nodes_[index];
  }
};