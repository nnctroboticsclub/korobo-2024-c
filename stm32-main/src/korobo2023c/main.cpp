#include <mbed.h>
#include <rtos.h>

#include <atomic>
#include <array>
#include <vector>
#include <sstream>

#include "../identify.h"
#include "../dcan.hpp"
#include "korobo2023c/2023c.hpp"

#include "../robotics/component/swerve/swerve.hpp"
#include "../robotics/sensor/gyro/bno055.hpp"
#include "../robotics/node/BLDC.hpp"
#include "../robotics/types/angle_joystick_2d.hpp"
#include "../robotics/assembly/dummy_motor_with_encoder.hpp"

#include "upper.hpp"

using namespace std::chrono_literals;

using namespace rtos;

#include "bus/driving_can.hpp"
#include "components/swerve.hpp"

using robotics::filter::PID;

class Communication {
 public:
  struct Config {
    struct {
      int id;
      int freqency;

      PinName rx, tx;
    } can;

    struct {
      PinName rx, tx;
    } driving_can;

    korobo::n2023c::Controller::Config controller_ids;

    korobo::n2023c::ValueStoreMain<float>::Config value_store_ids;

    struct {
      PinName sda;
      PinName scl;
    } i2c;

    struct {
      PinName swerve_pin_m0;
      PinName swerve_pin_m1;
      PinName swerve_pin_m2;
    } swerve_esc_pins;
  };

 public:
  DistributedCAN can_;
  std::unique_ptr<DrivingCANBus> driving_;

  korobo::n2023c::Controller controller_status_;
  korobo::n2023c::ValueStoreMain<float> value_store_;

  robotics::sensor::gyro::BNO055 gyro_;
  robotics::node::BLDC bldc[3];

 private:
  void InitCAN() {
    printf("\e[1;32m|\e[m \e[32m-\e[m Initializing CAN (Com)\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Initializing CAN Driver\n");
    can_.Init();
    driving_->Init();
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Adding Handlers\n");
    can_.OnEvent(0x40, [this](std::vector<uint8_t> data) {  //
      if (data.size() < 1) {
        printf("C< Invaid (0 bytes)\n");
        return;
      }
      std::stringstream ss;
      for (auto byte : data) {
        ss << std::setw(2) << std::hex << (int)byte << " ";
      }
      printf("C< %s(%d bytes)\n", ss.str().c_str(), data.size());
      controller_status_.Pass(data);
    });
    can_.OnEvent(0x61, [this](std::vector<uint8_t> data) {  //
      value_store_.Pass(data);
    });
  }

  void InitGyro() {
    SetStatus(DistributedCAN::Statuses::kInitializingGyro);
    printf("\e[1;32m|\e[m \e[32m-\e[m Initializing Gyro\n");
    auto gyro_init_status = gyro_.Init();
    if (!gyro_init_status) {
      printf(
          "\e[1;32m|\e[m \e[32m-\e[m \e[1;31m=> !!!\e[m BNO055 Detection "
          "failed.\e[m\n");
    }
  }

  void InitBLDC() {
    SetStatus(DistributedCAN::Statuses::kInitializingESC);
    printf("\e[1;32m|\e[m \e[32m-\e[m Initializing ESC\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Pulsing Max Pulsewidth\n");
    bldc[0].Init0();
    bldc[1].Init0();
    bldc[2].Init0();
    ThisThread::sleep_for(2s);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Pulsing Min Pulsewidth\n");
    bldc[0].Init1();
    bldc[1].Init1();
    bldc[2].Init1();
    ThisThread::sleep_for(1s);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m OK\n");
  }

 public:
  Communication(Config &config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        driving_(std::make_unique<DrivingCANBus>(new ikarashiCAN_mk2(
            config.driving_can.rx, config.driving_can.tx, 0))),
        controller_status_(config.controller_ids),
        value_store_(config.value_store_ids),
        gyro_(config.i2c.sda, config.i2c.scl),
        bldc{
            {config.swerve_esc_pins.swerve_pin_m0, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m1, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m2, 1000, 2000},
        } {}

  void SendNonReactiveValues() {
    this->driving_->Tick();
    this->driving_->Send();
  }

  void Init() {
    InitCAN();
    InitBLDC();
    InitGyro();

    printf("\e[1;32m|\e[m \e[32m-\e[m Setting Initial value\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Set Dummy value\n");
    controller_status_.swerve.angle_out.SetValue({0.1, 0.1});
    controller_status_.swerve.move.SetValue({0.1, 0.1});
    ThisThread::sleep_for(10ms);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Reset Initial value\n");
    controller_status_.swerve.angle_out.SetValue({0, 0});
    controller_status_.swerve.move.SetValue({0, 0});
  }

  void SetStatus(DistributedCAN::Statuses status) { can_.SetStatus(status); }

  void LinkToSwerve(SwerveComponent &swerve) {
    gyro_.Link(swerve.swerve_.robot_angle);

    swerve.swerve_.motors[0]->drive_.Link(bldc[0]);
    swerve.swerve_.motors[0]->steer_.output.Link(
        driving_->GetSwerveRot0().GetMotor());

    swerve.swerve_.motors[1]->drive_.Link(bldc[1]);
    swerve.swerve_.motors[1]->steer_.output.Link(
        driving_->GetSwerveRot1().GetMotor());

    swerve.swerve_.motors[2]->drive_.Link(bldc[2]);
    swerve.swerve_.motors[2]->steer_.output.Link(
        driving_->GetSwerveRot2().GetMotor());
  }
};

class App {
 public:
  struct Config {
    Communication::Config com;

    robotics::component::Swerve::Config swerve_config;

    bool swerve_origin_setting;
    bool encoder_debug;
    bool gain_debug;
  };

 private:
 private:
  Config config_;
  std::unique_ptr<Communication> com_;

  //* Robotics components
  // Driver

  mbed::DigitalOut emc;

  // Controller/ValueStore

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

        if (config_.gain_debug)
          printf("Rot Gain: %6.4lf %6.4lf %6.4lf\n",
                 swerve_->swerve_.motors[0]->steer_.output.GetValue(),
                 swerve_->swerve_.motors[1]->steer_.output.GetValue(),
                 swerve_->swerve_.motors[2]->steer_.output.GetValue());

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

using namespace robotics::component;

int main_sm() {
  ikarashiCAN_mk2 can{PB_5, PB_6, 0};
  ikakoMDC mtr[] = {
      ikakoMDC{1, -200, 200, 0.001, 0.0, 2.7, 0, 0.000015, 0.01},
      ikakoMDC{2, -200, 200, 0.001, 0.0, 2.7, 0, 0.000015, 0.01},
      ikakoMDC{3, -200, 200, 0.001, 0.0, 2.7, 0, 0.000015, 0.01},
      ikakoMDC{4, -200, 200, 0.001, 0.0, 2.7, 0, 0.000015, 0.01},
  };
  ikakoMDC_sender sender{mtr, 4, &can, 1};

  mtr[0].set_speed(-100);
  mtr[1].set_speed(-50);
  mtr[2].set_speed(50);
  mtr[3].set_speed(100);

  while (1) {
    sender.send();

    ThisThread::sleep_for(10ms);
  }
  return 0;
}
int main_im() {
  ikarashiCAN_mk2 can{PB_5, PB_6, 0};
  MDC mdc{&can, 0};

  mdc.GetNode(0).GetMotor().SetValue(-1);
  mdc.GetNode(1).GetMotor().SetValue(-0.5);
  mdc.GetNode(2).GetMotor().SetValue(0.5);
  mdc.GetNode(3).GetMotor().SetValue(1);
  while (1) {
    mdc.Tick();
    ThisThread::sleep_for(10ms);
  }

  return 0;
}
int main_itm() {
  ikarashiCAN_mk2 can{PB_5, PB_6, 0};
  DrivingCANBus *bus = new DrivingCANBus(&can);
  int i;

  bus->Init();

  bus->GetSwerveRot0().GetMotor().SetValue(-1);
  while (1) {
    printf("%6.4f\n", (i % 200) / 200.0f * 2 - 1);
    bus->GetSwerveRot0().GetMotor().SetValue((i % 200) / 200.0f * 2 - 1);

    bus->mdc0_.Send();
    i++;
    ThisThread::sleep_for(5ms);
  }

  return 0;
}

#include <ws2812B.hpp>

int main_led1() {
  Mikami::WS2812B ws2812b(PB_5, false);
  while (1) {
    ws2812b.Reset();
    ws2812b.Clear(7);
    ws2812b.Write(0xff0000);
    ws2812b.Write(0x00ff00);
    ws2812b.Write(0x0000ff);
    ws2812b.Write(0xff0000);
    ws2812b.Write(0x00ff00);
    ws2812b.Write(0x0000ff);
    ws2812b.Write(0x000000);
    ThisThread::sleep_for(100ms);
  }
}
int main_led2() {
  mbed::SPI serial(PB_5, NC, NC);
  serial.frequency(6.4E6);

  char buf[] = {
      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,

      0xff, 0xff, 0xff,
  };

  std::vector<char> reset;
  reset.resize(48);
  for (size_t i = 0; i < reset.size(); i++) {
    reset[i] = 0;
  }

  const size_t data_len = 8 * sizeof(buf);  // 1byte = 8bit => 2byte
  std::vector<char> data(data_len);
  for (size_t i = 0; i < data_len; i++) {
    data[i] = 0;
  }

  for (size_t i = 0; i < sizeof(buf); i++) {
    for (size_t j = 0; j < 8; j++) {
      data[i * 8 + j] =  //
          ((buf[i] >> (7 - j)) & 1) ? 0xF8 : 0xE0;
    }
  }

  std::vector<char> payload;
  payload.insert(payload.end(), reset.begin(), reset.end());
  payload.insert(payload.end(), data.begin(), data.end());

  for (size_t i = 0; i < payload.size(); i++) {
    printf("%02x ", payload[i]);
    if (i % 24 == 23) {
      printf("\n");
    }
  }
  printf("\n");

  printf("Waiting for 1ms\n");
  ThisThread::sleep_for(1ms);
  printf("Sending\n");
  serial.write(payload.data(), payload.size(), nullptr, 0);
  printf("Sent\n");

  return 0;
}
int main_can() {
  auto *can = new DistributedCAN(1, PB_8, PB_9, 1000000);
  printf("Init!\n");
  can->Init();
  printf("Setting up handlers\n");
  can->OnRx([](uint16_t id, std::vector<uint8_t> data) {
    printf("%3hX Received: ", id);
    for (auto byte : data) {
      printf("%02x ", byte);
    }
    printf("\n");
  });
  can->OnTx([](uint16_t id, std::vector<uint8_t> data) {
    printf("%3hX Sent: ", id);
    for (auto byte : data) {
      printf("%02x ", byte);
    }
    printf("\n");
  });

  printf("Sending\n");
  auto ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);
  printf("Sending\n");
  ret = can->Send(0x00, {0x00, 0x01, 0x02});
  printf("Result = %d\n", ret);

  while (1) {
    ThisThread::sleep_for(100s);
  }
}
int main_pro() {
  App::Config config{
      .com =
          {
              .can =
                  {
                      .id = CAN_ID,
                      .freqency = (int)1E6,
                      .rx = PB_8,
                      .tx = PB_9,
                  },
              .driving_can =
                  {
                      .rx = PB_5,
                      .tx = PB_6,
                  },
              .controller_ids =
                  {.swerve =
                       (controller::swerve::SwerveController::Config){
                           .joystick_id = 0,
                           .rot_right_45_id = 0,
                           .rot_left_45_id = 1,
                           .rotation_pid_enabled_id = 1,
                           .motor_0_pid_id = 0,
                           .motor_1_pid_id = 1,
                           .motor_2_pid_id = 2,
                           .angle_pid_id = 3,
                       },
                   .shot_joystick_id = 2,
                   .do_shot_id = 2,
                   .shot_speed_id = 0,
                   .max_elevation_id = 1},
              .value_store_ids =
                  {.swerve =
                       (controller::swerve::SwerveValueStore<float>::Config){
                           .motor_0_encoder_id = 0,
                           .motor_1_encoder_id = 1,
                           .motor_2_encoder_id = 2}},
              .i2c =
                  {
                      .sda = PC_9,
                      .scl = PA_8,
                  },
              .swerve_esc_pins =
                  {
                      // PA_9
                      // PB_10
                      .swerve_pin_m0 = PB_13,
                      .swerve_pin_m1 = PB_14,
                      .swerve_pin_m2 = PB_15,
                  },

          },

      .swerve_config = {.angle_offsets = {0, 120, 240}},

      .swerve_origin_setting = false,
      .encoder_debug = false,
      .gain_debug = true,
  };

  printf("Ctor\n");
  App app(config);
  printf("Init\n");
  app.Init();
  printf("end-\n");

  while (1) {
    ThisThread::sleep_for(100s);
  }

  return 0;
}

int main() {
  printf("main() started CAN_ID=%d\n", CAN_ID);

  printf("Build information:\n");
  printf("  - Build date: %s\n", __DATE__);
  printf("  - Build time: %s\n", __TIME__);
  printf("  - Analytics:\n");
  printf("    - sizeof(App): %d\n", sizeof(App));

  // main_sm();
  // main_im();
  // main_itm();
  // main_led1();
  // main_led2();
  // main_can();
  main_pro();
  return 0;
}
