#pragma once

#include <atomic>

#include <mbed.h>

#include "communication.hpp"
#include "components/upper.hpp"
#include "components/swerve.hpp"

#include "neopixel.hpp"

class App {
 public:
  struct Config {
    Communication::Config com;

    robotics::component::Swerve::Config swerve_config;

    bool swerve_origin_setting;
    bool encoder_debug;

    bool can1_debug;
  };

 private:
 private:
  Config config_;
  std::unique_ptr<Communication> com_;

  //* Robotics components
  bool emc_ctrl;
  bool emc_keep_alive;
  mbed::DigitalOut emc;

  //* Components
  std::unique_ptr<SwerveComponent> swerve_;
  korobo2023c::Upper upper_;

  //* Thread
  Thread *thr1;  //* Main
  Thread *thr2;  //* NeoPixel
  Thread *thr3;  //* Report

  std::atomic<bool> prevent_swerve_update;

  NeoPixel led_strip{PB_2, 20};

  void UpdateEMC() {
    auto emc_flag = (emc_ctrl && emc_keep_alive) ? true : false;
    printf("EMC --> %s\n", emc_flag ? "True" : "False");
    // emc = emc_ctrl;
    emc = emc_flag;
  }

  void DoReport() {
    swerve_->ReportTo(com_->can_);
    com_->Report();

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
      if (!prevent_swerve_update) {
        swerve_->swerve_.Update(0.01f);
      }
      upper_.Update(0.01f);

      if (i % 100 == 0) {  // interval: 100ms = 0.100s

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
  void ReportThread() {
    while (1) {
      com_->SendNonReactiveValues();
      DoReport();

      ThisThread::sleep_for(1ms);
    }
  }

  void NeoPixelThread() {
    NeoPixel led(PB_2, 20);

    int i = 0;
    int j = 0;

    Color buf[20] = {0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                     0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                     0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
                     0x000000, 0x000000, 0x000000, 0x000000, 0x00ff00};

    led.Clear();

    led.PutPixel((20 + i + 0) % 20, 0x00ff00);
    led.PutPixel((20 + i + 1) % 20, 0x00ff00);
    led.PutPixel((20 + i + 2) % 20, 0x00ff00);
    led.PutPixel((20 + i + 3) % 20, 0x00ff00);
    led.PutPixel((20 + i + 4) % 20, 0x00ff00);
    led.PutPixel((20 + i + 5) % 20, 0x00ff00);
    led.PutPixel((20 + i + 6) % 20, 0x00ff00);
    led.PutPixel((20 + i + 7) % 20, 0x00ff00);
    led.PutPixel((20 + i + 8) % 20, 0x00ff00);
    led.PutPixel((20 + i + 9) % 20, 0x00ff00);
    led.PutPixel((20 + i + 10) % 20, 0x00ff00);
    led.PutPixel((20 + i + 11) % 20, 0x00ff00);
    led.PutPixel((20 + i + 12) % 20, 0x00ff00);
    led.PutPixel((20 + i + 13) % 20, 0x00ff00);
    led.PutPixel((20 + i + 14) % 20, 0x00ff00);
    led.PutPixel((20 + i + 15) % 20, 0x00ff00);
    led.PutPixel((20 + i + 16) % 20, 0x00ff00);
    led.PutPixel((20 + i + 17) % 20, 0x00ff00);
    led.PutPixel((20 + i + 18) % 20, 0x00ff00);
    led.PutPixel((20 + i + 19) % 20, 0x00ff00);

    while (1) {
      led.Write();

      // auto tick = 20 - 1 * i / 40.0f;
      // int tick_int = tick < 0 ? 0 : (int)tick;

      // ThisThread::sleep_for(tick_int * 1ms);
      // i++;

      // for (int k = 0; k < 19; k++) {
      //   buf[k] = (buf[k] * 39 + buf[k + 1]) / 40;
      // }
    }
  }

 public:
  App(Config &config)
      : config_(config),
        com_(std::make_unique<Communication>(config.com)),
        emc(PC_1),
        swerve_(std::make_unique<SwerveComponent>(
            config.swerve_config, com_->controller_status_.swerve,
            com_->value_store_.swerve)),
        upper_(com_->controller_status_.upper) {
    prevent_swerve_update.store(false);

    com_->LinkToSwerve(*swerve_);

    com_->LinkToUpper(upper_);

    com_->controller_status_.steer_0_inverse.OnFire(
        [this]() { swerve_->InverseSteerMotor(0); });

    com_->controller_status_.steer_1_inverse.OnFire(
        [this]() { swerve_->InverseSteerMotor(1); });

    com_->controller_status_.steer_2_inverse.OnFire(
        [this]() { swerve_->InverseSteerMotor(2); });

    com_->controller_status_.soft_emc.SetChangeCallback([this](bool emc) {
      printf("EMC setted to %d\n", emc);
      this->emc_ctrl = !emc;
      this->UpdateEMC();
    });

    this->com_->can_.OnKeepAliveLost([this]() {
      this->emc_keep_alive = 0;
      this->UpdateEMC();
    });
    this->com_->can_.OnKeepAliveRecovered([this]() {
      this->emc_keep_alive = 1;
      this->UpdateEMC();
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

    printf("\e[1;32m|\e[m \e[32m-\e[m Starting NeoPixel Thread\n");
    thr2 = new Thread(osPriorityNormal, 1024 * 4);
    thr2->start(callback(this, &App::NeoPixelThread));

    printf("\e[1;32m|\e[m \e[32m-\e[m Starting Report Thread\n");
    thr3 = new Thread(osPriorityNormal, 1024 * 4);
    thr3->start(callback(this, &App::ReportThread));

    emc = 1;
    com_->Init();
    if (config_.can1_debug) com_->AddCAN1Debug();
    if (config_.swerve_origin_setting) InitSwerveOrigin();

    this->swerve_->Reset();

    printf("\e[1;32m+\e[m   \e[33m+\e[m\n");

    com_->SetStatus(DistributedCAN::Statuses::kReady);
  }
};