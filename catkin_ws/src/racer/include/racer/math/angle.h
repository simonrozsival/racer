#pragma once

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>

namespace racer::math {

struct angle {
private:
  double radians_;

public:
  angle(double radians) : radians_{radians} {}

  angle(const angle &other) = default;
  angle &operator=(const angle &other) = default;

  angle(angle &&other) = default;
  angle &operator=(angle &&other) = default;

  operator double() const { return radians_; }

  inline bool operator==(const angle &other) const {
    return radians_ == other.radians_;
  }

  inline bool operator<(const angle &other) const {
    return radians_ < other.radians_;
  }

  inline angle distance_to(const angle other) const {
    const double a = to_normal_angle();
    const double b = other.to_normal_angle();
    return std::min(a - b, 2 * M_PI - (a - b));
  }

  inline angle to_normal_angle() const {
    double r = radians_;
    while (r < 0)
      r += 2 * M_PI;
    while (r >= 2 * M_PI)
      r -= 2 * M_PI;
    return r;
  }

  inline double to_degrees() const { return radians_ / M_PI * 180.0; }

  static angle from_degrees(const double deg) {
    return angle(deg / 180.0 * M_PI);
  }
};

inline angle operator*(const double scale, const angle alpha) {
  return scale * double(alpha);
}

inline angle operator*(const angle alpha, const double scale) {
  return scale * double(alpha);
}

} // namespace racer::math
