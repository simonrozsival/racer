#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <algorithm>
#include <iostream>
#include <vector>

#include "racer/math/angle.h"

namespace racer::math {

struct vector {
private:
  double x_, y_;

public:
  constexpr vector() : x_{0}, y_{0} {}

  constexpr vector(double x, double y) : x_{x}, y_{y} {}

  vector(double length, angle direction)
      : x_{length * std::cos(direction)}, y_{length * std::sin(direction)} {}

  vector(vector &&vec) = default;
  vector &operator=(vector &&vec) = default;

  vector(const vector &vec) = default;
  vector &operator=(const vector &vec) = default;

  constexpr double x() const { return x_; }

  constexpr double y() const { return y_; }

  inline double dot(const vector &other) const {
    return x_ * other.x_ + y_ * other.y_;
  }

  vector normal() const {
    double size = length();
    return vector(-y_ / size, x_ / size);
  }

  vector normalized() const {
    double size = length();
    return vector(x_ / size, y_ / size);
  }

  bool operator==(const vector &other) const {
    return x_ == other.x_ && y_ == other.y_;
  }

  bool operator!=(const vector &other) const {
    return x_ != other.x_ && y_ != other.y_;
  }

  vector rotate(double radians) const {
    return vector(x_ * std::cos(radians) - y_ * std::sin(radians),
                  y_ * std::cos(radians) + x_ * std::sin(radians));
  }

  vector operator-(const vector &other) const {
    return vector(x_ - other.x_, y_ - other.y_);
  }

  vector operator+(const vector &other) const {
    return vector(x_ + other.x_, y_ + other.y_);
  }

  vector &operator+=(const vector &other) {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }

  constexpr bool operator<(const vector &other) const {
    return x_ < other.x_ && y_ < other.y_;
  }

  vector interpolate_with(const vector &other, const double weight,
                          const double weight_other) const {
    const double wa = weight / (weight + weight_other);
    const double wb = weight_other / (weight + weight_other);
    return vector(wa * x_ + wb * other.x_, wa * y_ + wb * other.y_);
  }

  inline double length_sq() const { return dot(*this); }

  inline double length() const { return std::sqrt(length_sq()); }

  inline double distance_sq(const vector &other) const {
    const double dx = x_ - other.x_;
    const double dy = y_ - other.y_;
    return dx * dx + dy * dy;
  }

  inline double distance(const vector &other) const {
    return std::sqrt(distance_sq(other));
  }

  inline double angle() const { return std::atan2(y_, x_); }

  friend std::ostream &operator<<(std::ostream &os, const vector &m) {
    return os << "[" << m.x_ << ", " << m.y_ << "]";
  }
};

inline vector operator*(const double scale, const vector vec) {
  return vector(scale * vec.x(), scale * vec.y());
}

inline vector operator*(const vector vec, const double scale) {
  return scale * vec;
}

inline vector operator/(const vector vec, const double scale) {
  return vec * (1.0 / scale);
}

} // namespace racer::math
