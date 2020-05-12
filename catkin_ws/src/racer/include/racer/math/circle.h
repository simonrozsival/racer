#pragma once

#include <iostream>
#include <vector>

#include "racer/math/angle.h"
#include "racer/math/point.h"
#include "racer/math/vector.h"

namespace racer::math {

struct circle {
private:
  racer::math::point center_;
  double radius_;

public:
  circle(const point &center, double radius)
      : center_(center), radius_(radius) {}

  circle(const circle &other) = default;
  circle &operator=(const circle &other) = default;

  circle(circle &&other) = default;
  circle &operator=(circle &&other) = default;

  const std::vector<point>
  points_on_circumference(double starting_angle, double arc_angle,
                          size_t number_of_points) const {
    std::vector<point> points;
    angle step = arc_angle / (double)number_of_points;

    for (double angle = starting_angle; angle < starting_angle + arc_angle;
         angle += step) {
      points.emplace_back(center_.x() + radius_ * std::cos(angle),
                          center_.y() + radius_ * std::sin(angle));
    }

    return points;
  }

  const point center() const { return center_; }
  const double radius() const { return radius_; }

  bool overlaps_with(const circle &other) const {
    return distance_sq(other) < std::pow(radius_ + other.radius_, 2);
  }

  bool contains(const point &pt) const {
    return center_.distance_sq(pt) < std::pow(radius_, 2);
  }

  inline double distance(const circle &other) const {
    return center_.distance(other.center_);
  }

  inline double distance_sq(const circle &other) const {
    return center_.distance_sq(other.center_);
  }

  bool operator==(const circle &other) const {
    // for the purposes of this algorithm, we assume that two circles
    // are equal, if they mostly overlap
    double dist_sq = distance_sq(other);
    return dist_sq <= std::pow(radius_, 2) ||
           dist_sq <= std::pow(other.radius_, 2);
  }
};

}