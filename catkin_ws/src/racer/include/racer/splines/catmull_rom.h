#pragma once

#include <vector>
#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/math.h"

namespace racer::splines::catmull_rom {

// Enumerate the segment between control points p1 and p2. p0 and p3 are the
// previous and the next control points in the series. See
// https://en.wikipedia.org/wiki/Centripetal_Catmull%E2%80%93Rom_spline and
// https://qroph.github.io/2018/07/30/smooth-paths-using-catmull-rom-splines.html
std::vector<racer::math::point>
enumerate_segment(racer::math::point p0, racer::math::point p1,
                  racer::math::point p2, racer::math::point p3, double step) {
  // We'll determine the number of pts from the euclidean distance between the
  // points of the segment. It's far from perfect but it will give us a good
  // enough sampling.
  std::size_t count = std::ceil(p2.distance(p1) / step);

  std::vector<racer::math::point> points;
  points.reserve(count);

  // const auto t01 = std::sqrt(p0.distance(p1));
  // const auto t12 = std::sqrt(p1.distance(p2));
  // const auto t23 = std::sqrt(p2.distance(p3));

  // const auto m1 = p2 - p1 + t12 * ((p1 - p0) / t01 - (p2 - p0) / (t01 +
  // t12)); const auto m2 = p2 - p1 + t12 * ((p3 - p2) / t23 - (p3 - p1) / (t12
  // + t23));

  const double alpha = 0.5;
  const double tension = 0.0;

  const double t0 = 0.0;
  const double t1 = t0 + std::pow(p0.distance(p1), alpha);
  const double t2 = t1 + std::pow(p1.distance(p2), alpha);
  const double t3 = t2 + std::pow(p2.distance(p3), alpha);

  const auto m1 =
      (1.0f - tension) * (t2 - t1) *
      ((p1 - p0) / (t1 - t0) - (p2 - p0) / (t2 - t0) + (p2 - p1) / (t2 - t1));
  const auto m2 =
      (1.0f - tension) * (t2 - t1) *
      ((p2 - p1) / (t2 - t1) - (p3 - p1) / (t3 - t1) + (p3 - p2) / (t3 - t2));

  const auto a = 2.0 * (p1 - p2) + m1 + m2;
  const auto b = -3.0 * (p1 - p2) - m1 - m1 - m2;
  const auto c = m1;
  const auto d = p1;

  for (std::size_t i = 0; i < count; ++i) {
    // we want `t` to start at 0 and end at 1
    const auto t = double(i) / double(count - 1);
    const auto t3 = t * t * t;
    const auto t2 = t * t;

    const auto pt = a * t3 + b * t2 + c * t + d;
    points.push_back(pt);
  }

  return points;
}

} // namespace racer::splines::catmull_rom