#pragma once

#include "robotics/filter/pid.hpp"
#include "robotics/filter/angled_motor.hpp"
#include "robotics/filter/inc_angled_motor.hpp"
#include "robotics/filter/muxer.hpp"
#include "robotics/node/motor.hpp"

namespace korobo2023c {
using PID = robotics::filter::PID<float>;
using AngledMotor = robotics::filter::AngledMotor<float>;
using IncAngledMotor = robotics::filter::IncAngledMotor<float>;
using Motor = robotics::node::Motor<float>;
using Muxer = robotics::filter::Muxer<float>;

template <typename T>
using Node = robotics::Node<T>;

class Upper {
 public:
  struct Controller {
    struct Config {
      uint8_t shot_joystick_id;
      uint8_t do_shot_id;
      uint8_t do_load_id;
      uint8_t shot_speed_id;
      uint8_t load_speed_id;
      uint8_t max_elevation_id;

      uint8_t revolver_change_id;

      uint8_t elevation_pid_id;
      uint8_t rotation_pid_id;
      uint8_t shot_l_factor_id;
      uint8_t shot_r_factor_id;
    };

    controller::JoyStick shot;           // upper
    controller::Boolean do_shot;         // upper
    controller::Boolean do_load;         // upper
    controller::Float shot_speed;        // upper
    controller::Float load_speed;        // upper
    controller::Float max_elevation;     // upper
    controller::Action revolver_change;  // upper

    controller::PID elevation_pid;    // upper
    controller::PID rotation_pid;     // upper
    controller::Float shot_l_factor;  // upper
    controller::Float shot_r_factor;  // upper

    Controller(Config const& config = {})
        : shot(config.shot_joystick_id),
          do_shot(config.do_shot_id),
          do_load(config.do_load_id),
          shot_speed(config.shot_speed_id),
          load_speed(config.load_speed_id),
          max_elevation(config.max_elevation_id),
          revolver_change(config.revolver_change_id),
          elevation_pid(config.elevation_pid_id),
          rotation_pid(config.rotation_pid_id),
          shot_l_factor(config.shot_l_factor_id),
          shot_r_factor(config.shot_r_factor_id)

    {}

    bool Pass(controller::RawPacket const& packet) {
      return shot.Pass(packet) || do_shot.Pass(packet) ||
             do_load.Pass(packet) || shot_speed.Pass(packet) ||
             load_speed.Pass(packet) || max_elevation.Pass(packet) ||
             revolver_change.Pass(packet) || elevation_pid.Pass(packet) ||
             rotation_pid.Pass(packet) || shot_l_factor.Pass(packet) ||
             shot_r_factor.Pass(packet);
    }
  };

  AngledMotor elevation_motor;
  AngledMotor rotation_motor;
  IncAngledMotor revolver;

  std::unique_ptr<Motor> shot_r;
  std::unique_ptr<Motor> shot_l;

  Node<float> load;

 private:
  Controller& controller;

  Node<float> zero;

  Node<float> load_speed;
  Node<float> shot_speed;

  Muxer load_mux;
  Muxer shot_mux;

 private:
  float max_elevation_angle = 60.0;

 public:
  Upper(Controller& ctrl) : controller(ctrl) {
    printf("[Upper] Init\n");
    zero.SetValue(0);

    load_speed.SetValue(0.5);
    shot_speed.SetValue(0.5);

    load_mux.AddInput(zero);
    load_mux.AddInput(load_speed);
    load_mux.output_.Link(load);

    shot_mux.AddInput(zero);
    shot_mux.AddInput(shot_speed);
    shot_mux.output_.SetChangeCallback([this](float speed) {
      shot_l->SetValue(speed);
      shot_r->SetValue(-speed);
    });
  }

  void LinkController() {
    controller.shot.SetChangeCallback([this](robotics::JoyStick2D x) {
      printf("S< %6.4f %6.4f\n", x[0], x[1]);
      SetRotationAngle(x[0] * 60);
      SetElevationAngle(x[1] * 60);
    });

    controller.do_shot.SetChangeCallback([this](bool shot) {
      printf("[Ctrl::Upper] Set ShotState -> %d\n", shot);
      SetShotState(shot);
    });

    controller.do_load.SetChangeCallback([this](bool load) {
      printf("[Ctrl::Upper] Set LoadState -> %d\n", load);
      SetLoadState(load);
    });

    controller.shot_speed.SetChangeCallback(
        [this](float speed) { SetShotSpeed(speed); });
    controller.load_speed.Link(load_speed);

    controller.max_elevation.SetChangeCallback(
        [this](float angle) { SetMaxElevationAngle(angle); });
    controller.revolver_change.OnFire([this]() { RevolverChange(); });

    controller.elevation_pid.Link(elevation_motor.pid.gains);
    controller.rotation_pid.Link(rotation_motor.pid.gains);

    controller.shot_l_factor.Link(shot_l->factor);
    controller.shot_r_factor.Link(shot_r->factor);
  }

  void Update(float dt) {
    elevation_motor.Update(dt);
    rotation_motor.Update(dt);
    revolver.Update(dt);
  }

  // 仰角
  void SetElevationAngle(float angle) {
    elevation_motor.goal.SetValue(
        angle > max_elevation_angle ? max_elevation_angle : angle);
  }

  // 最大仰角
  void SetMaxElevationAngle(float angle) {
    max_elevation_angle = angle;
    if (elevation_motor.goal.GetValue() > angle) {
      elevation_motor.goal.SetValue(angle);
    }
  }

  // 横の角
  void SetRotationAngle(float angle) { rotation_motor.goal.SetValue(angle); }

  // 発射速度
  void SetShotSpeed(float cent) { shot_speed.SetValue(cent); }

  void SetShotState(bool do_shot) { shot_mux.Select(do_shot ? 1 : 0); }

  /**
   * @param load do load motor rolling
   */
  void SetLoadState(bool load) { load_mux.Select(load ? 1 : 0); }

  // リロード
  void RevolverChange() { revolver.AddAngle(360); }
};
}  // namespace korobo2023c