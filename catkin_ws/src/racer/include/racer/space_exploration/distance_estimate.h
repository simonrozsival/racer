#pragma once

#include <iostream>

#include "racer/space_exploration/circle_node.h"

namespace racer::space_exploration {

struct distance_estimate {
  bool operator()(const std::shared_ptr<circle_node> &a,
                  const std::shared_ptr<circle_node> &b) const {
    return a->distance_estimate > b->distance_estimate;
  }
};

}