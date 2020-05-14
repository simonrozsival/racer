#pragma once

#include <algorithm>
#include <vector>

#include "racer/math/circle.h"

namespace racer::space_exploration {

struct circle_node {
  const racer::math::circle examined_circle;
  const double distance_from_start;
  const double distance_estimate;
  const double heading_angle;
  std::shared_ptr<circle_node> parent;

  circle_node(racer::math::circle examined_circle, double distance,
              double distance_to_go, double heading_angle,
              std::shared_ptr<circle_node> parent)
      : examined_circle(examined_circle), distance_from_start(distance),
        distance_estimate(distance + distance_to_go),
        heading_angle(heading_angle), parent(parent) {}

  std::vector<racer::math::circle> reconstruct_path() {
    std::vector<racer::math::circle> path;

    path.push_back(examined_circle);
    auto node = parent;

    while (node) {
      path.push_back(node->examined_circle);
      node = node->parent;
    }

    std::reverse(std::begin(path), std::end(path));
    return path;
  }
};

} // namespace racer::space_exploration
