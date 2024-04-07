#pragma once

#include <array>
#include <ikarashiCAN_mk2.h>

#include <robotics/assembly/ikakoMDC.hpp>

#include <robotics/network/dcan.hpp>

class MDC {
  ::ikakoMDC motors_[4];
  ikakoMDC_sender sender_;
  std::array<robotics::assembly::ikakoMDCPair<float>, 4> motor_nodes_;
  ikarashiCAN_mk2 *linked_ican_;

  int report_counter = 0;

  void ReportSpeed(robotics::network::DistributedCAN &can, uint8_t id);

  void ReportEncoder(robotics::network::DistributedCAN &can, uint8_t id);

 public:
  MDC(ikarashiCAN_mk2 *can, int mdc_id);

  void Tick();

  void ReportTo(robotics::network::DistributedCAN &can, uint8_t id);

  int Send();

  robotics::assembly::MotorPair<float> &GetNode(int index);
};