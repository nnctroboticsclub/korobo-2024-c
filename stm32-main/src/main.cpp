#include <mbed.h>
#include <rtos.h>

#include <array>
#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"
#include "korobo2023c/2023c.hpp"

#include "robotics/component/swerve/swerve.hpp"
#include "robotics/sensor/gyro/bno055.hpp"
#include "robotics/node/BLDC.hpp"
#include "robotics/types/angle_joystick_2d.hpp"
#include "robotics/assembly/ikakoMDC.hpp"
#include "robotics/assembly/dummy_motor_with_encoder.hpp"

#include "./upper.hpp"

using namespace std::chrono_literals;

using namespace rtos;

using robotics::filter::PID;

class MDC {
  ::ikakoMDC motors_[4];
  ikakoMDC_sender sender_;
  std::array<robotics::assembly::ikakoMDCPair<float>, 4> motor_nodes_;
  ikarashiCAN_mk2 *linked_ican_;

  robotics::assembly::DummyMotorWithEncoder<float> d;

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

  int Send() {
    auto ret = sender_.send();
    if (ret == 0) {
      printf("MDC: Sending the command is failed.\n");
    }
    return ret;
  }

  robotics::assembly::MotorWithEncoder<float> &GetNode(int index) {
    return motor_nodes_[index];
  }
};

class DrivingCANBus {
  ikarashiCAN_mk2 *ican_;

 public:
  MDC mdc0_;
  MDC mdc1_;
  MDC mdc2_;

  DrivingCANBus(ikarashiCAN_mk2 *ican)
      : ican_(ican), mdc0_(ican, 1), mdc1_(ican, 2), mdc2_(ican, 3) {}

  void Init() { ican_->read_start(); }

  void Tick() {
    mdc0_.Tick();
    // mdc1_.Tick();
    // mdc2_.Tick();
  }

  robotics::assembly::MotorWithEncoder<float> &GetSwerveRot0() {
    return mdc0_.GetNode(0);
  }
  robotics::assembly::MotorWithEncoder<float> &GetSwerveRot1() {
    return mdc0_.GetNode(1);
  }
  robotics::assembly::MotorWithEncoder<float> &GetSwerveRot2() {
    return mdc0_.GetNode(2);
  }
  robotics::assembly::MotorWithEncoder<float> &GetShotL() {
    return mdc0_.GetNode(3);
  }

  robotics::assembly::MotorWithEncoder<float> &GetRevolver() {
    return mdc1_.GetNode(0);
  }
  robotics::assembly::MotorWithEncoder<float> &GetLoad() {
    return mdc1_.GetNode(1);
  }
  robotics::assembly::MotorWithEncoder<float> &GetHorizontal() {
    return mdc1_.GetNode(2);
  }
  robotics::assembly::MotorWithEncoder<float> &GetElevation() {
    return mdc1_.GetNode(3);
  }

  robotics::assembly::MotorWithEncoder<float> &GetShotR() {
    return mdc2_.GetNode(0);
  }
};

struct SwerveComponent {
  robotics::component::Swerve swerve_;
  controller::swerve::SwerveController &ctrl_;
  controller::swerve::SwerveValueStore<float> &values_;

  void Link_() {
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

  void ReportTo(DistributedCAN &can) {
    std::vector<uint8_t> physical_report(8);
    physical_report.reserve(8);
    physical_report[0] = 0x00;

    int i = 0;
    for (auto &motor : swerve_.motors) {
      auto angle_power = motor->steer_.output.GetValue();
      auto mag = motor->drive_.GetValue();

      physical_report[1 + i * 2] = (uint8_t)std::min((int)(mag * 255.0f), 255);
      physical_report[2 + i * 2] =
          (uint8_t)std::min((int)(angle_power * 255.0f), 255);

      i++;
    }
    physical_report[7] = swerve_.robot_angle.GetValue() / 360.0 * 255;

    auto ret = can.Send(0xa0, physical_report);
    if (ret != 1) {
      printf("Swerve Report: Sending the report is failed.\n");
    }
  }

  SwerveComponent(robotics::component::Swerve::Config swerve_config,
                  controller::swerve::SwerveController &b,
                  controller::swerve::SwerveValueStore<float> &c)
      : swerve_(swerve_config), ctrl_(b), values_(c) {
    Link_();
  }
};

class App {
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

    struct {
      PinName sda;
      PinName scl;
    } i2c;

    struct {
      PinName swerve_pin_m0;
      PinName swerve_pin_m1;
      PinName swerve_pin_m2;
    } swerve_esc_pins;

    robotics::component::Swerve::Config swerve_config;
    korobo::n2023c::Controller::Config controller_ids;
    korobo::n2023c::ValueStoreMain<float>::Config value_store_ids;
  };

 private:
 private:
  //* Robotics components
  // Driver
  DistributedCAN can_;
  std::unique_ptr<DrivingCANBus> driving_;

  robotics::node::BLDC bldc[3];
  robotics::sensor::gyro::BNO055 gyro_;
  mbed::DigitalOut emc;

  // Controller/ValueStore
  korobo::n2023c::Controller controller_status_;
  korobo::n2023c::ValueStoreMain<float> value_store_;

  //* Components
  std::unique_ptr<SwerveComponent> swerve_;
  korobo2023c::Upper upper_;

  //* Thread
  Thread *thr1;  //* 無関係

  void DoReport() {
    this->swerve_->ReportTo(can_);

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

    auto ret = can_.Send(0xa0, pid_report);
    if (ret != 1) {
      printf("PID Report: Sending the report is failed.\n");
    }
  }

  void MainThread() {
    int i = 0;
    while (1) {
      this->driving_->mdc0_.Send();

      swerve_->swerve_.Update(0.01f);
      if (i % 10 == 0) {  // interval: 100ms = 0.100s
        this->DoReport();

        i = 0;
      }
      i++;

      ThisThread::sleep_for(1ms);
    }
  }

 public:
  App(Config &config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        driving_(std::make_unique<DrivingCANBus>(new ikarashiCAN_mk2(
            config.driving_can.rx, config.driving_can.tx, 0))),
        bldc{
            {config.swerve_esc_pins.swerve_pin_m0, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m1, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m2, 1000, 2000},
        },
        gyro_(config.i2c.sda, config.i2c.scl),
        emc(PC_1),
        controller_status_(config.controller_ids),
        value_store_(config.value_store_ids),
        swerve_(std::make_unique<SwerveComponent>(config.swerve_config,
                                                  controller_status_.swerve,
                                                  value_store_.swerve)) {
    printf("CTor joined\n");

    printf("CTor joined - 1\n");
    gyro_.Link(swerve_->swerve_.robot_angle);
    printf("CTor joined - 2\n");
    swerve_->swerve_.motors[0]->drive_.Link(bldc[0]);
    printf("CTor joined - 3\n");
    swerve_->swerve_.motors[0]->steer_.output.Link(
        driving_->GetSwerveRot0().GetMotor());
    printf("CTor joined - 4\n");

    swerve_->swerve_.motors[1]->drive_.Link(bldc[1]);
    printf("CTor joined - 5\n");
    swerve_->swerve_.motors[1]->steer_.output.Link(
        driving_->GetSwerveRot1().GetMotor());
    printf("CTor joined - 6\n");

    swerve_->swerve_.motors[2]->drive_.Link(bldc[2]);
    printf("CTor joined - 7\n");
    swerve_->swerve_.motors[2]->steer_.output.Link(
        driving_->GetSwerveRot2().GetMotor());
    printf("CTor joined - 8\n");

    {
      auto &motor = this->driving_->GetElevation();
      motor.GetEncoder() >> upper_.elevation_motor.feedback;
      upper_.elevation_motor.output >> motor.GetMotor();
    }
    {
      auto &motor = this->driving_->GetHorizontal();
      motor.GetEncoder() >> upper_.rotation_motor.feedback;
      upper_.rotation_motor.output >> motor.GetMotor();
    }
    {
      auto &motor = this->driving_->GetRevolver();
      motor.GetEncoder() >> upper_.revolver.encoder;
      upper_.revolver.output >> motor.GetMotor();
    }
    upper_.shot.SetChangeCallback([this](float speed) {
      this->driving_->GetShotL().GetMotor().SetValue(speed);
      this->driving_->GetShotR().GetMotor().SetValue(-speed);
    });

    this->controller_status_.shot_speed.SetChangeCallback(
        [this](float speed) { upper_.SetShotSpeed(speed); });
    this->controller_status_.max_elevation.SetChangeCallback(
        [this](float angle) { upper_.SetMaxElevationAngle(angle); });
    this->controller_status_.shot.SetChangeCallback(
        [this](robotics::JoyStick2D x) {
          upper_.SetRotationAngle(x[0]);
          upper_.SetElevationAngle(x[1]);
        });
  }

  void Init() {
    printf("\e[1;32m-\e[m Init\n");
    printf("\e[1;32m|\e[m \e[32m1\e[m Starting Main Thread\n");
    this->thr1 = new Thread(osPriorityNormal, 1024 * 4);
    this->thr1->start(callback(this, &App::MainThread));
    printf("\e[1;32m|\e[m \e[32m2\e[m Initializing Gyro\n");
    auto gyro_init_status = gyro_.Init();
    if (!gyro_init_status) {
      printf(
          "\e[1;32m|\e[m \e[32m2\e[m \e[1;31m=> !!!\e[m BNO055 Detection "
          "failed.\e[m\n");
    }
    printf("\e[1;32m|\e[m \e[32m3\e[m Initializing CAN\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m1\e[m Initializing CAN Driver\n");
    can_.Init();
    this->driving_->Init();
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m2\e[m Adding Handlers\n");
    can_.OnEvent(0x40, [this](std::vector<uint8_t> data) {  //
      std::stringstream ss;
      for (auto byte : data) {
        ss << std::setw(2) << std::hex << (int)byte << " ";
      }
      printf("C< %s\n", ss.str().c_str());
      controller_status_.Parse(data);
    });
    can_.OnEvent(0x61, [this](std::vector<uint8_t> data) {  //
      value_store_.Parse(data);
    });

    printf("\e[1;32m|\e[m \e[32m4\e[m Setting Initial value\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m1\e[m Set Dummy value\n");
    controller_status_.swerve.angle_out.SetValue({0.1, 0.1});
    controller_status_.swerve.move.SetValue({0.1, 0.1});
    ThisThread::sleep_for(10ms);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m2\e[m Reset Initial value\n");
    controller_status_.swerve.angle_out.SetValue({0, 0});
    controller_status_.swerve.move.SetValue({0, 0});
    printf("\e[1;32m|\e[m \e[32m5\e[m Initializing ESC\n");
    emc = 1;
    can_.SetStatus(DistributedCAN::Statuses::kInitializingESC);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m1\e[m Pulsing Max Pulsewidth\n");
    bldc[0].Init0();
    bldc[1].Init0();
    bldc[2].Init0();
    ThisThread::sleep_for(2s);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m2\e[m Pulsing Min Pulsewidth\n");
    bldc[0].Init1();
    bldc[1].Init1();
    bldc[2].Init1();
    ThisThread::sleep_for(2s);
    printf("\e[1;32m|\e[m \e[32m6\e[m Setting swerve motors to origin point\n");
    auto &motor0 = swerve_->swerve_.motors[0]->steer_;
    auto &motor1 = swerve_->swerve_.motors[1]->steer_;
    auto &motor2 = swerve_->swerve_.motors[2]->steer_;

    motor0.goal.SetValue(0);
    motor1.goal.SetValue(0);
    motor2.goal.SetValue(0);

    bool motor_0_initialized = false;
    bool motor_1_initialized = false;
    bool motor_2_initialized = false;

    motor0.output.SetValue(0.4);
    motor1.output.SetValue(0.4);
    motor2.output.SetValue(0.4);
    while (1) {
      if (!motor_0_initialized && motor0.feedback.GetValue() != 0) {
        ThisThread::sleep_for(2s);
        printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Motor 0 is ready\n");
        motor_0_initialized = true;
      }
      if (!motor_1_initialized && motor1.feedback.GetValue() != 0) {
        printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Motor 1 is ready\n");
        motor_1_initialized = true;
      }
      if (!motor_2_initialized && motor2.feedback.GetValue() != 0) {
        printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m-\e[m Motor 2 is ready\n");
        motor_2_initialized = true;
      }

      if (motor_0_initialized) motor0.Update(0.01f);
      if (motor_1_initialized) motor1.Update(0.01f);
      if (motor_2_initialized) motor2.Update(0.01f);

      ThisThread::sleep_for(10ms);

      break;  // THIS IS FOR DEBUG
    }

    printf("\e[1;32m+\e[m   \e[33m+\e[m\n");

    can_.SetStatus(DistributedCAN::Statuses::kReady);
  }
};

using namespace robotics::component;

int main_sm(int argc, char const *argv[]) {
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

int main_im(int argc, char const *argv[]) {
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
int main_(int argc, char const *argv[]) {
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

int main(int argc, char const *argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);

  printf("Build information:\n");
  printf("  - Build date: %s\n", __DATE__);
  printf("  - Build time: %s\n", __TIME__);
  printf("  - Analytics:\n");
  printf("    - sizeof(App): %d\n", sizeof(App));

  App::Config config{
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
      .swerve_config = {.angle_offsets = {0, 120, 240}},
      .controller_ids = {.swerve =
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
                         .shot_speed_id = 0,
                         .max_elevation_id = 1},
      .value_store_ids =
          {.swerve =
               (controller::swerve::SwerveValueStore<float>::Config){
                   .motor_0_encoder_id = 0,
                   .motor_1_encoder_id = 1,
                   .motor_2_encoder_id = 2}},
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