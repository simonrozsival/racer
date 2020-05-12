#pragma once

#include <algorithm>
#include <iostream>
#include <vector>

#include "racer/math/vector.h"
#include "racer/math/point.h"

namespace racer::math {

struct rectangle {
  const point A, B, C, D;

  rectangle(point a, point b, point c, point d) : A{a}, B{b}, C{c}, D{d} {}

  rectangle rotate(double radians) const {
    return {A.rotate(radians), B.rotate(radians), C.rotate(radians),
            D.rotate(radians)};
  }

  bool intersects(const rectangle &other) const {
    for (const auto &edge : edges()) {
      if (intersets_along(edge.normal(), other)) {
        return true;
      }
    }

    for (const auto &edge : other.edges()) {
      if (intersets_along(edge.normal(), other)) {
        return true;
      }
    }

    return false;
  }

private:
  std::vector<vector> edges() const { return {B - A, C - B, D - C, A - D}; }

  std::pair<double, double> project_onto(const vector &axis) const {
    double a = axis.dot(A);
    double b = axis.dot(B);
    double c = axis.dot(C);
    double d = axis.dot(D);

    return std::pair<double, double>(std::min({a, b, c, d}),
                                     std::max({a, b, c, d}));
  }

  bool intersets_along(const vector &axis, const rectangle &other) const {
    auto a = project_onto(axis);
    auto b = other.project_onto(axis);

    auto interval_distance =
        a.first < b.first ? b.first - a.second : a.first - b.second;

    return interval_distance <= 0;
  }
};

} // namespace racer::math
