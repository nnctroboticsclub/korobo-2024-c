#pragma once

namespace robotics {
namespace input {

template <typename T>
class IInputController {
 public:
  virtual T const &GetValue() const = 0;
  virtual void SetValue(T value) = 0;
};

template <typename T>
class Input {
 public:
  class Controller : public IInputController<T> {
    Input<T> *input_;

   public:
    Controller(Input<T> *input) : input_(input) {}

    T const &GetValue() const override { return input_->value_; }
    void SetValue(T value) override { input_->value_ = value; }
  };

 private:
  T value_;

 public:
  Input() : value_(0) {}
  Input(T value) : value_(value) {}

  Controller GetController() { return Controller(this); }

  T const &GetValue() const { return value_; }
};
}  // namespace input
}  // namespace robotics