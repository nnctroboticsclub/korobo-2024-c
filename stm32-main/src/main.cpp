#include <mbed.h>
#include <rtos.h>

#include <vector>
#include <sstream>

#include "identify.h"
#include "dcan.hpp"
#include "korobo2023c/2023c.hpp"

#include "robotics/component/swerve/swerve.hpp"
#include "robotics/sensor/gyro/bno055.hpp"
#include "robotics/node/ikakoMDC.hpp"
#include "robotics/node/BLDC.hpp"
#include "robotics/types/angle_joystick_2d.hpp"

using namespace std::chrono_literals;

using namespace rtos;

using robotics::filter::PID;

class App {
 public:
  struct Config {
    struct {
      int id;
      int freqency;

      PinName rx, tx;
    } can;

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
  DistributedCAN can_;
  robotics::sensor::gyro::BNO055 gyro_;
  korobo::n2023c::Controller controller_status_;
  korobo::n2023c::ValueStoreMain<float> value_store_;
  robotics::component::Swerve swerve_;
  robotics::node::BLDC bldc[3];
  robotics::node::ikakoMDCMotor motors[3];

  Thread thr_pool1;

  void ReportThread() {
    std::vector<uint8_t> physical_report(8);
    std::vector<uint8_t> pid_report(5);
    physical_report.reserve(8);
    pid_report.reserve(8);
    int c = 0;

    physical_report[0] = 0x00;
    pid_report[0] = 0x01;
    while (1) {
      ThisThread::sleep_for(10ms);
      int i = 0;
      for (auto &motor : swerve_.motors) {
        auto angle_power = motor->steer_.output.GetValue();
        auto mag = motor->drive_.GetValue();
        auto angle_error = motor->steer_.pid.CalculateError();

        physical_report[1 + i * 2] =
            (uint8_t)std::min((int)(mag * 255.0f), 255);
        physical_report[2 + i * 2] =
            (uint8_t)std::min((int)(angle_power * 255.0f), 255);

        pid_report[1 + i] = (uint8_t)std::max(
            std::min((int)(angle_error * 127.0f + 128), 255), 0);
        i++;
      }
      physical_report[7] = swerve_.robot_angle.GetValue() / 360.0 * 255;
      pid_report[4] = (uint8_t)std::min(
          (int)(swerve_.angle.CalculateError() / 360 * 255.0f), 255);

      if (0)
        printf(
            "(%4.1f %4.1f)"
            "\e[31m|\e[m"
            "(%4.1f %4.1f)"
            "\e[31m|\e[m"
            "(%4.1f %4.1f)"
            "\e[41m \e[m"

            "(%4.1f %4.1f)"
            "\e[31m|\e[m"
            "%4.1f"
            "\e[41m \e[m"

            "\n",
            controller_status_.shot.GetValue()[0],
            controller_status_.shot.GetValue()[1],

            controller_status_.swerve.angle.GetValue().magnitude,
            controller_status_.swerve.angle.GetValue().angle,

            controller_status_.swerve.move.GetValue()[0],
            controller_status_.swerve.move.GetValue()[1],
            //
            swerve_.move_ctrl.GetValue()[0], swerve_.move_ctrl.GetValue()[1],
            swerve_.angle_ctrl.GetValue());
      if (0)
        printf(
            "(%4.1f %4.1f) (%4.1f %4.1f) (%4.1f %4.1f), "
            "(%4.1f %4.1f %4.1f) %4.1f"
            "\n",
            swerve_.motors[0]->steer_.output.GetValue(),
            swerve_.motors[0]->drive_.GetValue(),
            swerve_.motors[1]->steer_.output.GetValue(),
            swerve_.motors[1]->drive_.GetValue(),
            swerve_.motors[2]->steer_.output.GetValue(),
            swerve_.motors[2]->drive_.GetValue(),

            swerve_.motors[0]->steer_.pid.CalculateError(),
            swerve_.motors[1]->steer_.pid.CalculateError(),
            swerve_.motors[2]->steer_.pid.CalculateError(),
            swerve_.angle.CalculateError()  //
        );
      if (0)
        printf(
            "(g:%4.1f f:%4.1f o:%4.1f) (g:%4.1f f:%4.1f o:%4.1f) (g:%4.1f "
            "f:%4.1f o:%4.1f)"
            "\n",
            swerve_.motors[0]->steer_.goal.GetValue(),
            swerve_.motors[0]->steer_.feedback.GetValue(),
            swerve_.motors[0]->steer_.output.GetValue(),
            swerve_.motors[1]->steer_.goal.GetValue(),
            swerve_.motors[1]->steer_.feedback.GetValue(),
            swerve_.motors[1]->steer_.output.GetValue(),
            swerve_.motors[2]->steer_.goal.GetValue(),
            swerve_.motors[2]->steer_.feedback.GetValue(),
            swerve_.motors[2]->steer_.output.GetValue());
      if (0) printf("\n");

      auto ret = can_.Send(0xa0, physical_report);
      if (ret != 1) {
        printf("ReportThread()/CanSend() failed (A)\n");
      }
      ret = can_.Send(0xa0, pid_report);
      if (ret != 1) {
        printf("ReportThread()/CanSend() failed (B)\n");
      }
    }
  }

 public:
  App(Config &config)
      : can_(config.can.id, config.can.rx, config.can.tx, config.can.freqency),
        gyro_(config.i2c.sda, config.i2c.scl),
        controller_status_(config.controller_ids),
        value_store_(config.value_store_ids),
        swerve_(config.swerve_config),
        bldc{
            {config.swerve_esc_pins.swerve_pin_m0, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m1, 1000, 2000},
            {config.swerve_esc_pins.swerve_pin_m2, 1000, 2000},
        },
        motors{
            {0, -50, 50},
            {1, -50, 50},
            {2, -50, 50},
        } {
    controller_status_.swerve.angle_pid.Link(swerve_.angle.gains);
    controller_status_.swerve.motor_0_pid.Link(
        swerve_.motors[0]->steer_.pid.gains);
    controller_status_.swerve.motor_1_pid.Link(
        swerve_.motors[1]->steer_.pid.gains);
    controller_status_.swerve.motor_2_pid.Link(
        swerve_.motors[2]->steer_.pid.gains);

    controller_status_.swerve.move.Link(swerve_.move_ctrl);
    controller_status_.swerve.angle.SetChangeCallback(
        [this](robotics::AngleStick2D angle) {
          swerve_.angle_ctrl.SetValue(angle.angle > 180 ? angle.angle - 360
                                                        : angle.angle);
        });
    gyro_.Link(swerve_.robot_angle);

    controller_status_.swerve.rotation_pid_enabled.SetChangeCallback(
        [this](bool enabled) { swerve_.SetAnglePID(enabled); });

    // swerve fb link
    value_store_.swerve.motor_0_encoder.Link(
        swerve_.motors[0]->steer_.feedback);
    value_store_.swerve.motor_1_encoder.Link(
        swerve_.motors[1]->steer_.feedback);
    value_store_.swerve.motor_2_encoder.Link(
        swerve_.motors[2]->steer_.feedback);

    // swerve out link
    swerve_.motors[0]->drive_.Link(bldc[0]);
    swerve_.motors[0]->steer_.output.Link(motors[0]);

    swerve_.motors[1]->drive_.Link(bldc[1]);
    swerve_.motors[1]->steer_.output.Link(motors[1]);

    swerve_.motors[2]->drive_.Link(bldc[2]);
    swerve_.motors[2]->steer_.output.Link(motors[2]);
  }
  //
  void Init() {
    printf("\e[1;32m-\e[m Init\n");
    printf("\e[1;32m|\e[m \e[32m1\e[m Initializing Gyro\n");
    auto gyro_init_status = gyro_.Init();
    if (!gyro_init_status) {
      printf(
          "\e[1;32m|\e[m \e[32m1\e[m \e[1;31m=> !!!\e[m BNO055 Detection "
          "failed.\e[m\n");
    }
    printf("\e[1;32m|\e[m \e[32m2\e[m Initializing CAN\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m1\e[m Initializing CAN Driver\n");
    can_.Init();
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

    printf("\e[1;32m|\e[m \e[32m3\e[m Setting Initial value\n");
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m1\e[m Set Dummy value\n");
    controller_status_.swerve.angle.SetValue({0.1, 0.1});
    controller_status_.swerve.move.SetValue({0.1, 0.1});
    ThisThread::sleep_for(10ms);
    printf("\e[1;32m|\e[m \e[32m|\e[m \e[33m2\e[m Reset Initial value\n");
    controller_status_.swerve.angle.SetValue({0, 0});
    controller_status_.swerve.move.SetValue({0, 0});
    printf("\e[1;32m|\e[m \e[32m4\e[m Initializing ESC\n");
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
    printf("\e[1;32m|\e[m \e[32m5\e[m Setting swerve motors to origin point\n");
    if (0) {
      auto &motor0 = swerve_.motors[0]->steer_;
      auto &motor1 = swerve_.motors[1]->steer_;
      auto &motor2 = swerve_.motors[2]->steer_;

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
    }
    printf("\e[1;32m+\e[m   \e[33m+\e[m\n");
    can_.SetStatus(DistributedCAN::Statuses::kReady);
  }

  void Main() {
    thr_pool1.start(callback(this, &App::ReportThread));
    while (1) {
      swerve_.Update(0.01f);
      ThisThread::sleep_for(10ms);
    }
  }
};

using namespace robotics::component;

int main(int argc, char const *argv[]) {
  printf("main() started CAN_ID=%d\n", CAN_ID);

  App::Config config{
      .can =
          {
              .id = CAN_ID,
              .freqency = (int)1E6,
              .rx = PB_8,
              .tx = PB_9,
          },
      .i2c =
          {
              .sda = PC_9,
              .scl = PA_8,
          },
      .swerve_esc_pins =
          {
              .swerve_pin_m0 = PB_13,
              .swerve_pin_m1 = PB_14,
              .swerve_pin_m2 = PB_15,
          },
      .swerve_config = {.angle_offsets = {0, 120, 240}},
      .controller_ids = {.swerve =
                             (controller::swerve::SwerveController::Config){
                                 .joystick_id = 0,
                                 .angle_joystick_id = 1,
                                 .rotation_pid_enabled_id = 1,
                                 .motor_0_pid_id = 0,
                                 .motor_1_pid_id = 1,
                                 .motor_2_pid_id = 2,
                                 .angle_pid_id = 3,
                             }},
      .value_store_ids =
          {.swerve =
               (controller::swerve::SwerveValueStore<float>::Config){
                   .motor_0_encoder_id = 0,
                   .motor_1_encoder_id = 1,
                   .motor_2_encoder_id = 2}},
  };

  App app(config);
  app.Init();
  app.Main();

  return 0;
}