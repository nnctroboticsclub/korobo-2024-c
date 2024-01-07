namespace robotics::filter {
template <typename T>
class AngleNormalizer {
  T prev_input = 0;
  T value = 0;

 public:
  T Update(T input) {
    T delta_rot_y_deg = input - prev_input;
    prev_input = input;

    if (delta_rot_y_deg > 180)
      delta_rot_y_deg -= 360;
    else if (delta_rot_y_deg < -180)
      delta_rot_y_deg += 360;

    value += delta_rot_y_deg;

    return value;
  }
};
}  // namespace robotics::filter