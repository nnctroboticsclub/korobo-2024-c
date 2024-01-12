#include "../node/node.hpp"

namespace robotics::filter {
template <typename T>
class AngleNormalizer {
  T prev_input = 0;
  T value = 0;

 public:
  Node<T> input;
  Node<T> output;

  void Update(float power = 1) {
    T input_value = input.GetValue();

    T delta_rot_y_deg = input_value - prev_input;
    prev_input = input_value;

    if (delta_rot_y_deg > 180)
      delta_rot_y_deg -= 360;
    else if (delta_rot_y_deg < -180)
      delta_rot_y_deg += 360;

    value += delta_rot_y_deg * power;

    if (power == 0) {
      value = 0;
    }

    output.SetValue(value);
  }
};
}  // namespace robotics::filter