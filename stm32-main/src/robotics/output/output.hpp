#pragma once
namespace robotics::output {

template <typename T>
class Output {
 private:
  T value_;

  void Update(T value) virtual;

 public:
  void SetOutputValue(T value) {
    value_ = value;
    Update(value);
  }

  T GetOutputValue() { return value_; }
};

}  // namespace robotics::output