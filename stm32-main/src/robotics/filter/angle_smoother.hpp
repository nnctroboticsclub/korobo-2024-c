namespace robotics::filter {
template <typename T>
class AngleNormalizer {
  T prev_input = 0;
  T value = 0;

 public:
  T Update(T input, float power = 1) {
    T delta_rot_y_deg = input - prev_input;
    prev_input = input;

    if (delta_rot_y_deg > 180)
      delta_rot_y_deg -= 360;
    else if (delta_rot_y_deg < -180)
      delta_rot_y_deg += 360;

    value += delta_rot_y_deg * power;

    if (power == 0) {
      value = 0;
    }

    return value;
  }
};
}  // namespace robotics::filter