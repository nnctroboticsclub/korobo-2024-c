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

  void UpdateEMC();

  void DoReport();

  void MainThread();
  void ReportThread();
  void NeoPixelThread();

 public:
  App(Config &config);

  void InitSwerveOrigin();

  void Init();
};