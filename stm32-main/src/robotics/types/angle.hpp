#pragma once

namespace robotics {
inline namespace types {
enum class AngleType {
  kRadians,
  kDegrees,
};

template <typename T>
class BaseAngle {
 public:
  BaseAngle<T> operator+(BaseAngle<T> const& rhs) const {
    return BaseAngle<T>(value_ + rhs.value_);
  }

  BaseAngle<T> operator-(BaseAngle<T> const& rhs) const {
    return BaseAngle<T>(value_ - rhs.value_);
  }

  BaseAngle<T> operator*(T const& rhs) const {
    return BaseAngle<T>(value_ * rhs);
  }

  BaseAngle<T> operator/(T const& rhs) const {
    return BaseAngle<T>(value_ / rhs);
  }

  BaseAngle<T>& operator+=(BaseAngle<T> const& rhs) {
    value_ += rhs.value_;
    return *this;
  }

  BaseAngle<T>& operator-=(BaseAngle<T> const& rhs) {
    value_ -= rhs.value_;
    return *this;
  }

  BaseAngle<T>& operator*=(T const& rhs) {
    value_ *= rhs;
    return *this;
  }

  BaseAngle<T>& operator/=(T const& rhs) {
    value_ /= rhs;
    return *this;
  }
};
template <typename T, AngleType type>
class Angle {};

template <typename T>
class Angle<T, AngleType::kDegrees> : public BaseAngle<T> {
  T value_;

 public:
  Angle() : value_(0) {}

  Angle(T value) : value_(value) {}

  Angle<T, AngleType::kRadians> ToRadians() const {
    return Angle<T, AngleType::kRadians>(value_ * M_PI / 180);
  }

  T GetValue() const { return value_; }
};

template <typename T>
class Angle<T, AngleType::kRadians> : public BaseAngle<T> {
  T value_;

 public:
  Angle() : value_(0) {}

  Angle(T value) : value_(value) {}

  Angle<T, AngleType::kDegrees> ToDegrees() const {
    return Angle<T, AngleType::kDegrees>(value_ * 180 / M_PI);
  }

  T GetValue() const { return value_; }
};

}  // namespace types

}  // namespace robotics