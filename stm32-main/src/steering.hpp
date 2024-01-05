

class MotorInfo {
 public:
  Wheel wheel;

 public:
  float motor_angle_deg;  // deg
};

template <int N>
class Steering {
 public:
  std::array<MotorInfo, N> motors;
  Toggle toggle;

 private:
  PID<Vector2> move_pid = new Vector2PID(1.0f, 0.00f, 0.00f);
  PID<float> angle_pid = new PID(0.7f, 0.30f, 0.15f);

  void MoveAndRotate(Vector2 velocity_raw, float rotation_in_raw,
                     float angle_power) {
    float dt = Time.deltaTime;
    float move_to_factor = move_factor_slider.value;
    float rotation_factor = rotation_factor_slider.value;

    // Velocity PID
    Vector3 velocity3d = -GetComponent<Rigidbody>().velocity;
    Vector2 velocity_fb = new Vector2(velocity3d.x, velocity3d.z);
    Vector2 velocity = toggle.isOn
                           ? velocity_raw
                           : move_pid.Update(velocity_raw, velocity_fb, dt);

    // Rotation PID
    float rotation_in = rot_in_normalizer.Update(rotation_in_raw);
    float rotation_fb =
        self_rot_y_normalizer.Update(transform.localRotation.eulerAngles.y);
    float rotation = toggle.isOn
                         ? rotation_in
                         : angle_pid.Update(rotation_in, rotation_fb, dt);

    for (int i = 0; i < motors.Count; i++) {
      MotorInfo motor = motors[i];

      float normal_angle_deg = motor.motor_angle_deg + 90.0f;
      Vector2 rotation_vector =
          rotation / 90 *
          new Vector2(Mathf.Cos(normal_angle_deg * Mathf.Deg2Rad),
                      Mathf.Sin(normal_angle_deg * Mathf.Deg2Rad));

      Vector2 move_vector = 2.0f / motors.Count * velocity;

      Vector2 vector =
          move_to_factor * move_vector + rotation_factor * rotation_vector;
      vector *= power_factor_slider.value;

      motor.wheel.Power(vector);
    }
  }
};