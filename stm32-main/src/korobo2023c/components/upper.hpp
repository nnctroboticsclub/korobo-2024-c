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
  AngledMotor elevation_motor;
  AngledMotor rotation_motor;
  IncAngledMotor revolver;
  Node<float> shot;
  Node<float> load;

  Node<float> load_speed;

 private:
  Muxer load_mux;
  Node<float> zero;

 private:
  float max_elevation_angle = 60.0;
  float shot_speed = 0.5;
  bool in_shot = 0;

 public:
  Upper() {
    printf("[Upper] Init\n");
    zero.SetValue(0);

    load_mux.AddInput(zero);
    load_mux.AddInput(load_speed);

    load_mux.output_.Link(load);
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
  void SetShotSpeed(float cent) {
    shot_speed = cent;
    if (in_shot) shot.SetValue(shot_speed);
  }

  void Shot() {
    in_shot = true;
    shot.SetValue(shot_speed);
  }
  void ShotStop() {
    in_shot = false;
    shot.SetValue(0);
  }

  /**
   * @param load do load motor rolling
   */
  void SetLoadState(bool load) { load_mux.Select(load ? 1 : 0); }

  // リロード
  void RevolverChange() { revolver.AddAngle(360); }
};
}  // namespace korobo2023c