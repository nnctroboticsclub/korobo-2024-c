#pragma once

#include <chrono>

#include <mbed.h>

#include "robotics/filter/pid.hpp"
#include "robotics/filter/angled_motor.hpp"
#include "robotics/filter/inc_angled_motor.hpp"
#include "robotics/filter/muxer.hpp"
#include "robotics/node/motor.hpp"

#include "../../controller/joystick.hpp"
#include "../../controller/boolean.hpp"
#include "../../controller/float.hpp"
#include "../../controller/pid.hpp"

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

      uint8_t revolver_pid_id;
    };

    controller::JoyStick shot;            // upper
    controller::Boolean do_shot;          // upper
    controller::Boolean do_load;          // upper
    controller::Float shot_speed;         // upper
    controller::Float load_speed;         // upper
    controller::Float max_elevation;      // upper
    controller::Boolean revolver_change;  // upper

    controller::Float shot_l_factor;  // upper
    controller::Float shot_r_factor;  // upper

    controller::PID revolver_pid;  // uppe

    Controller(Config const& config = {})
        : shot(config.shot_joystick_id),
          do_shot(config.do_shot_id),
          do_load(config.do_load_id),
          shot_speed(config.shot_speed_id),
          load_speed(config.load_speed_id),
          max_elevation(config.max_elevation_id),
          revolver_change(config.revolver_change_id),
          shot_l_factor(config.shot_l_factor_id),
          shot_r_factor(config.shot_r_factor_id),
          revolver_pid(config.revolver_pid_id)

    {}

    bool Pass(controller::RawPacket const& packet) {
      return shot.Pass(packet) || do_shot.Pass(packet) ||
             do_load.Pass(packet) || shot_speed.Pass(packet) ||
             load_speed.Pass(packet) || max_elevation.Pass(packet) ||
             revolver_change.Pass(packet) || shot_l_factor.Pass(packet) ||
             shot_r_factor.Pass(packet) || revolver_pid.Pass(packet);
    }
  };

  AngledMotor elevation_motor;
  AngledMotor rotation_motor;
  // IncAngledMotor revolver;
  std::unique_ptr<Motor> revolver;

  std::unique_ptr<Motor> shot_r;
  std::unique_ptr<Motor> shot_l;

  Node<float> load;

 private:
  Controller& ctrl;

  Node<float> zero;
  Node<float> neg_half;

  Node<float> load_speed;
  Node<float> shot_speed;

  Muxer load_mux;
  Muxer shot_mux;

  mbed::Timer timer;

  enum class LoadState {
    kIdle,
    kInRotation,
    kInReversing,
  };
  LoadState load_state = LoadState::kIdle;

  const std::chrono::duration<float> revolver_reversing_time = 30ms;

 private:
  float max_elevation_angle = 60.0;

 public:
  Upper(Controller& ctrl) : ctrl(ctrl) {
    printf("[Upper] Init\n");
    zero.SetValue(0);
    neg_half.SetValue(0.5);

    load_speed.SetValue(-0.18);
    shot_speed.SetValue(0.25);

    load_mux.AddInput(zero);
    load_mux.AddInput(load_speed);
    load_mux.AddInput(neg_half);
    load_mux.output_.Link(load);

    shot_mux.AddInput(zero);
    shot_mux.AddInput(shot_speed);
    shot_mux.output_.SetChangeCallback([this](float speed) {
      shot_l->SetValue(speed);
      shot_r->SetValue(-speed);
    });
  }

  void LinkController() {
    ctrl.shot.SetChangeCallback([this](robotics::JoyStick2D x) {
      printf("S< %6.4f %6.4f\n", x[0], x[1]);
      SetRotationAngle(x[0]);
      SetElevationAngle(x[1]);
    });

    ctrl.do_shot.SetChangeCallback([this](bool shot) {
      printf("[Ctrl::Upper] Set ShotState -> %d\n", shot);
      SetShotState(shot);
    });

    ctrl.do_load.SetChangeCallback([this](bool load) {
      printf("[Ctrl::Upper] Set LoadState -> %d\n", load);
      SetLoadState(load ? LoadAction::kStartRotation
                        : LoadAction::kStopRotation);
    });

    ctrl.shot_speed.SetChangeCallback(
        [this](float speed) { SetShotSpeed(speed); });
    ctrl.load_speed.Link(load_speed);

    ctrl.max_elevation.SetChangeCallback(
        [this](float angle) { SetMaxElevationAngle(angle); });
    ctrl.revolver_change.SetChangeCallback([this](bool revolver) {
      if (revolver) {
        RevolverChange(RevolverAction::kStartRotation);
      } else {
        RevolverChange(RevolverAction::kStopRotation);
      }
    });

    ctrl.shot_l_factor.Link(shot_l->factor);
    ctrl.shot_r_factor.Link(shot_r->factor);

    // controller.revolver_pid.Link(revolver.pid.gains);
  }

  void Update(float dt) {
    elevation_motor.Update(dt);
    rotation_motor.Update(dt);
    //     revolver.Update(dt);

    switch (load_state) {
      case LoadState::kInRotation:
        break;

      case LoadState::kInReversing:
        if (timer.elapsed_time() > revolver_reversing_time) {
          SetLoadState(LoadAction::kStopCounterRotation);
        }
        break;

      case LoadState::kIdle:
        break;
    }
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
  enum class LoadAction {
    kStartRotation,
    kStopRotation,
    kStartCounterRotation,
    kStopCounterRotation,
  };
  void SetLoadState(LoadAction state) {
    switch (state) {
      case LoadAction::kStartRotation:
        load_state = LoadState::kInRotation;
        load_mux.Select(1);
        break;

      case LoadAction::kStopRotation:
        SetLoadState(LoadAction::kStartCounterRotation);
        break;

      case LoadAction::kStartCounterRotation:
        load_state = LoadState::kInReversing;
        load_mux.Select(2);

        timer.reset();
        timer.start();
        break;

      case LoadAction::kStopCounterRotation:
        load_state = LoadState::kIdle;
        load_mux.Select(0);
        break;
    }
  }

  // リロード
  enum class RevolverAction {
    kStartRotation,
    kStopRotation,
  };
  void RevolverChange(RevolverAction state) {
    switch (state) {
      case RevolverAction::kStartRotation:
        revolver->SetValue(0.5);
        break;

      case RevolverAction::kStopRotation:
        revolver->SetValue(0);
        break;
    }
  }
};
}  // namespace korobo2023c