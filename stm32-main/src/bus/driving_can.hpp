#pragma once

#include <ikarashiCAN_mk2.h>
#include <robotics/assembly/motor_pair.hpp>
#include <robotics/network/dcan.hpp>

#include <robotics/registry/ikako_mdc.hpp>

class DrivingCANBus {
  ikarashiCAN_mk2 *ican_;
  int report_counter = 0;

 public:
  robotics::registry::ikakoMDC mdc0_;
  robotics::registry::ikakoMDC mdc1_;
  robotics::registry::ikakoMDC mdc2_;

  DrivingCANBus(ikarashiCAN_mk2 *ican);

  void Init();
  void Tick();
  void Send();

  void ReportTo(robotics::network::DistributedCAN &can);

  robotics::assembly::MotorPair<float> &GetSwerveRot0();
  robotics::assembly::MotorPair<float> &GetSwerveRot1();
  robotics::assembly::MotorPair<float> &GetSwerveRot2();
  robotics::assembly::MotorPair<float> &GetRevolver();

  robotics::assembly::MotorPair<float> &GetShotR();
  robotics::assembly::MotorPair<float> &GetShotL();
  robotics::assembly::MotorPair<float> &GetHorizontal();
  robotics::assembly::MotorPair<float> &GetElevation();

  robotics::assembly::MotorPair<float> &GetLoad();
};