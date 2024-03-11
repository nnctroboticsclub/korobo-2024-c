#pragma once

#include "../../robotics/component/swerve/swerve.hpp"
#include "../../controller/swerve.hpp"

#include "../../dcan.hpp"

struct SwerveComponent {
  robotics::component::Swerve swerve_;
  controller::swerve::SwerveController &ctrl_;
  controller::swerve::SwerveValueStore<float> &values_;

  int report_counter = 0;

  void Link_() {
    ctrl_.angle_pid.SetValue(swerve_.angle.gains.GetValue());
    ctrl_.motor_0_pid.SetValue(swerve_.motors[0]->steer_.pid.gains.GetValue());
    ctrl_.motor_1_pid.SetValue(swerve_.motors[1]->steer_.pid.gains.GetValue());
    ctrl_.motor_2_pid.SetValue(swerve_.motors[2]->steer_.pid.gains.GetValue());

    ctrl_.angle_pid.Link(swerve_.angle.gains);
    ctrl_.motor_0_pid.Link(swerve_.motors[0]->steer_.pid.gains);
    ctrl_.motor_1_pid.Link(swerve_.motors[1]->steer_.pid.gains);
    ctrl_.motor_2_pid.Link(swerve_.motors[2]->steer_.pid.gains);
    ctrl_.move.Link(swerve_.move_ctrl);

    ctrl_.angle_out.SetChangeCallback([this](robotics::AngleStick2D angle) {
      swerve_.angle_ctrl.SetValue(angle.angle > 180 ? angle.angle - 360
                                                    : angle.angle);
    });
    // gyro_.Link(swerve_.robot_angle);

    ctrl_.rotation_pid_enabled.SetChangeCallback(
        [this](bool enabled) { swerve_.SetAnglePID(enabled); });

    // swerve fb link
    values_.motor_0_encoder.Link(swerve_.motors[0]->steer_.feedback);
    values_.motor_1_encoder.Link(swerve_.motors[1]->steer_.feedback);
    values_.motor_2_encoder.Link(swerve_.motors[2]->steer_.feedback);
  }

  void ReportMotor(DistributedCAN &can, int index) {
    std::vector<uint8_t> report(7);
    report.reserve(7);
    report[0] = 0x20 | index;
    report[1] =
        std::min((int)(swerve_.motors[index]->steer_.pid.gains.GetValue().p /
                       10.0f * 255.0f),
                 255);
    report[2] =
        std::min((int)(swerve_.motors[index]->steer_.pid.gains.GetValue().i /
                       10.0f * 255.0f),
                 255);
    report[3] =
        std::min((int)(swerve_.motors[index]->steer_.pid.gains.GetValue().d /
                       10.0f * 255.0f),
                 255);
    report[4] =
        std::min((int)(swerve_.motors[index]->steer_.pid.fb_.GetValue() /
                       360.0f * 255.0f),
                 255);

    auto output = swerve_.motors[index]->steer_.pid.output_.GetValue();
    output = std::min(std::max(output, -1.0f), 1.0f);

    report[5] = (output / 2 + 0.5) * 255.0f;

    auto ret = can.Send(0xa0, report);
    if (ret != 1) {
      printf("Swerve Report: Sending the report is failed.\n");
    }
  }

  void ReportSwerve(DistributedCAN &can) {
    std::vector<uint8_t> physical_report(8);
    physical_report.reserve(8);
    physical_report[0] = 0x00;

    int i = 0;
    for (auto &motor : swerve_.motors) {
      auto angle = motor->steer_.goal.GetValue();
      auto mag = motor->drive_.GetValue();

      while (angle > 360.0f) {
        angle -= 360.0f;
      }

      while (angle < 0.0f) {
        angle += 360.0f;
      }

      physical_report[1 + i * 2] = (uint8_t)std::min((int)(mag * 255.0f), 255);
      physical_report[2 + i * 2] =
          (uint8_t)std::min((int)(angle / 360.0f * 255.0f), 255);

      i++;
    }
    physical_report[7] = swerve_.robot_angle.GetValue() / 360.0 * 255;

    auto ret = can.Send(0xa0, physical_report);
    if (ret != 1) {
      printf("Swerve Report: Sending the report is failed.\n");
    }
  }

  void ReportTo(DistributedCAN &can) {
    if (report_counter == 0) {
      ReportMotor(can, 0);
      ReportSwerve(can);
      report_counter++;
    } else if (report_counter == 1) {
      ReportMotor(can, 1);
      ReportMotor(can, 2);
      // report_counter++;
      report_counter = 0;
    }
  }

  SwerveComponent(robotics::component::Swerve::Config swerve_config,
                  controller::swerve::SwerveController &b,
                  controller::swerve::SwerveValueStore<float> &c)
      : swerve_(swerve_config), ctrl_(b), values_(c) {
    Link_();
  }

  void Reset() { swerve_.Reset(); }
};