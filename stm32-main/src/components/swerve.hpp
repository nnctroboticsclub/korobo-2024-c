#pragma once

#include <robotics/component/swerve/swerve.hpp>
#include <robotics/controller/swerve.hpp>
#include <robotics/network/dcan.hpp>

struct SwerveComponent {
  robotics::component::Swerve swerve_;
  controller::swerve::SwerveController &ctrl_;
  controller::swerve::SwerveValueStore<float> &values_;

 private:
  int report_counter = 0;

  void Link_();

  void ReportMotor(robotics::network::DistributedCAN &can, int index);

  void ReportSwerve(robotics::network::DistributedCAN &can);

 public:
  SwerveComponent(robotics::component::Swerve::Config swerve_config,
                  controller::swerve::SwerveController &b,
                  controller::swerve::SwerveValueStore<float> &c);

  void ReportTo(robotics::network::DistributedCAN &can);

  void InverseSteerMotor(int index);

  void Reset();
};