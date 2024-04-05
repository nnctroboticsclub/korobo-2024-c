#include "app.hpp"

void App::DoReport() {
  swerve_->ReportTo(com_->can_);
  com_->Report();

  std::vector<uint8_t> pid_report(5);
  pid_report.reserve(5);
  pid_report[0] = 0x01;

  int i = 0;
  for (auto &motor : swerve_->swerve_.motors) {
    auto angle_error = motor->steer_.pid.CalculateError();

    pid_report[1 + i] =
        (uint8_t)std::max(std::min((int)(angle_error * 127.0f + 128), 255), 0);
    i++;
  }

  pid_report[4] = (uint8_t)std::min(
      (int)(swerve_->swerve_.angle.CalculateError() / 360 * 255.0f), 255);

  auto ret = com_->can_.Send(0xa0, pid_report);
  if (ret != 1) {
    printf("PID Report: Sending the report is failed.\n");
  }
}

void App::MainThread() {
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
void App::ReportThread() {
  while (1) {
    com_->SendNonReactiveValues();
    DoReport();

    ThisThread::sleep_for(1ms);
  }
}

void App::NeoPixelThread() {
  NeoPixel led(PB_2, 20);

  led.Clear();

  led.PutPixel(0, 0x00ff00);
  led.PutPixel(1, 0x00ff00);
  led.PutPixel(2, 0x00ff00);
  led.PutPixel(3, 0x00ff00);
  led.PutPixel(4, 0x00ff00);
  led.PutPixel(5, 0x00ff00);
  led.PutPixel(6, 0x00ff00);
  led.PutPixel(7, 0x00ff00);
  led.PutPixel(8, 0x00ff00);
  led.PutPixel(9, 0x00ff00);
  led.PutPixel(10, 0x00ff00);
  led.PutPixel(11, 0x00ff00);
  led.PutPixel(12, 0x00ff00);
  led.PutPixel(13, 0x00ff00);
  led.PutPixel(14, 0x00ff00);
  led.PutPixel(15, 0x00ff00);
  led.PutPixel(16, 0x00ff00);
  led.PutPixel(17, 0x00ff00);
  led.PutPixel(18, 0x00ff00);
  led.PutPixel(19, 0x00ff00);

  while (1) {
    led.Write();
  }
}

App::App(Config &config)
    : config_(config),
      com_(std::make_unique<Communication>(config.com)),
      emc(std::make_shared<robotics::utils::EMC>()),
      emc_out(PC_1),
      swerve_(std::make_unique<SwerveComponent>(config.swerve_config,
                                                com_->controller_status_.swerve,
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

  auto emc_ctrl = emc->AddNode();

  com_->controller_status_.soft_emc.SetChangeCallback(
      [this, emc_ctrl](bool emc) {
        printf("EMC(CTRL) setted to %d\n", emc);
        emc_ctrl->SetValue(!emc);
      });

  auto keep_alive = emc->AddNode();
  this->com_->can_.OnKeepAliveLost([keep_alive]() {
    printf("EMC(CAN) setted to %d\n", false);
    keep_alive->SetValue(false);
  });
  this->com_->can_.OnKeepAliveRecovered([keep_alive]() {
    printf("EMC(CAN) setted to %d\n", true);
    keep_alive->SetValue(true);
  });
}

void App::InitSwerveOrigin() {
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

void App::Init() {
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

  emc->output.Link(emc_out);

  emc->Init();

  com_->Init();
  if (config_.can1_debug) com_->AddCAN1Debug();
  if (config_.swerve_origin_setting) InitSwerveOrigin();

  this->swerve_->Reset();

  printf("\e[1;32m+\e[m   \e[33m+\e[m\n");

  com_->SetStatus(DistributedCAN::Statuses::kReady);
}