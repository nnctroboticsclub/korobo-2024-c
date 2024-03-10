#pragma once

#include <atomic>

#include <mbed.h>

#include "communication.hpp"
#include "components/upper.hpp"
#include "components/swerve.hpp"

class App {
 public:
  struct Config {
    Communication::Config com;

    robotics::component::Swerve::Config swerve_config;

    bool swerve_origin_setting;
    bool encoder_debug;
  };

 private:
 private:
  Config config_;
  std::unique_ptr<Communication> com_;

  //* Robotics components
  mbed::DigitalOut emc;

  //* Components
  std::unique_ptr<SwerveComponent> swerve_;
  korobo2023c::Upper upper_;

  //* Thread
  Thread *thr1;  //* 無関係

  std::atomic<bool> prevent_swerve_update;

  void DoReport() {
    swerve_->ReportTo(com_->can_);

    std::vector<uint8_t> pid_report(5);
    pid_report.reserve(5);
    pid_report[0] = 0x01;

    int i = 0;
    for (auto &motor : swerve_->swerve_.motors) {
      auto angle_error = motor->steer_.pid.CalculateError();

      pid_report[1 + i] = (uint8_t)std::max(
          std::min((int)(angle_error * 127.0f + 128), 255), 0);
      i++;
    }

    pid_report[4] = (uint8_t)std::min(
        (int)(swerve_->swerve_.angle.CalculateError() / 360 * 255.0f), 255);

    auto ret = com_->can_.Send(0xa0, pid_report);
    if (ret != 1) {
      printf("PID Report: Sending the report is failed.\n");
    }
  }

  void MainThread() {
    int i = 0;
    while (1) {
      com_->SendNonReactiveValues();

      if (!prevent_swerve_update) {
        swerve_->swerve_.Update(0.01f);
      }
      if (i % 10 == 0) {  // interval: 100ms = 0.100s
        DoReport();

        if (config_.encoder_debug)
          printf(" Encoder: %6.4lf %6.4lf %6.4lf\n",
                 com_->value_store_.swerve.motor_0_encoder.GetValue(),
                 com_->value_store_.swerve.motor_1_encoder.GetValue(),
                 com_->value_store_.swerve.motor_2_encoder.GetValue());

        i = 0;
      }
      i++;

      ThisThread::sleep_for(1ms);
    }
  }

 public:
  App(Config &config)
      : config_(config),
        com_(std::make_unique<Communication>(config.com)),
        emc(PC_1),
        swerve_(std::make_unique<SwerveComponent>(
            config.swerve_config, com_->controller_status_.swerve,
            com_->value_store_.swerve)) {
    prevent_swerve_update.store(false);

    com_->LinkToSwerve(*swerve_);

    {
      auto &motor = com_->driving_->GetElevation();
      motor.GetEncoder() >> upper_.elevation_motor.feedback;
      upper_.elevation_motor.output >> motor.GetMotor();
    }
    {
      auto &motor = com_->driving_->GetHorizontal();
      motor.GetEncoder() >> upper_.rotation_motor.feedback;
      upper_.rotation_motor.output >> motor.GetMotor();
    }
    {
      auto &motor = com_->driving_->GetRevolver();
      motor.GetEncoder() >> upper_.revolver.encoder;
      upper_.revolver.output >> motor.GetMotor();
    }
    upper_.shot.SetChangeCallback([this](float speed) {
      com_->driving_->GetShotL().GetMotor().SetValue(speed);
      com_->driving_->GetShotR().GetMotor().SetValue(-speed);
    });

    com_->controller_status_.shot_speed.SetChangeCallback(
        [this](float speed) { upper_.SetShotSpeed(speed); });
    com_->controller_status_.max_elevation.SetChangeCallback(
        [this](float angle) { upper_.SetMaxElevationAngle(angle); });
    com_->controller_status_.shot.SetChangeCallback(
        [this](robotics::JoyStick2D x) {
          printf("U< %6.4f %6.4f\n", x[0], x[1]);
          upper_.SetRotationAngle(x[0]);
          upper_.SetElevationAngle(x[1]);
        });
  }

  void InitSwerveOrigin() {
    auto &motor0 = swerve_->swerve_.motors[0]->steer_;
    auto &motor1 = swerve_->swerve_.motors[1]->steer_;
    auto &motor2 = swerve_->swerve_.motors[2]->steer_;

    bool motor_0_initialized = false;
    bool motor_1_initialized = false;
    bool motor_2_initialized = false;

    printf("\e[1;32m|\e[m \e[32m-\e[m Setting swerve motors to origin point\n");
    prevent_swerve_update = true;
    motor0.output.SetValue(0.4);
    motor1.output.SetValue(0.4);
    motor2.output.SetValue(0.4);
    while (1) {
      if (!motor_0_initialized && motor0.feedback.GetValue() != 0) {
        printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Motor 0 is ready\n");
        motor_0_initialized = true;
        motor0.goal.SetValue(0);
      }
      if (!motor_1_initialized && motor1.feedback.GetValue() != 0) {
        printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Motor 1 is ready\n");
        motor_1_initialized = true;
        motor1.goal.SetValue(0);
      }
      if (!motor_2_initialized && motor2.feedback.GetValue() != 0) {
        printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Motor 2 is ready\n");
        motor_2_initialized = true;
        motor2.goal.SetValue(0);
      }

      if (motor_0_initialized && motor_1_initialized && motor_2_initialized) {
        break;
      }

      if (motor_0_initialized) motor0.Update(0.01f);
      if (motor_1_initialized) motor1.Update(0.01f);
      if (motor_2_initialized) motor2.Update(0.01f);

      ThisThread::sleep_for(10ms);
    }
    prevent_swerve_update = false;
  }

  void Init() {
    printf("\e[1;32m-\e[m Init\n");
    printf("\e[1;32m|\e[m \e[32m-\e[m Starting Main Thread\n");
    thr1 = new Thread(osPriorityNormal, 1024 * 4);
    thr1->start(callback(this, &App::MainThread));

    emc = 1;
    com_->Init();
    if (config_.swerve_origin_setting) InitSwerveOrigin();

    printf("\e[1;32m+\e[m   \e[33m+\e[m\n");

    com_->SetStatus(DistributedCAN::Statuses::kReady);
  }
};