#pragma once

#include <iostream>

#include "racer/sehs/circle_node.h"

namespace racer::sehs {

struct distance_estimate {
  bool operator()(const std::shared_ptr<circle_node> &a,
                  const std::shared_ptr<circle_node> &b) const {
    return a->distance_estimate > b->distance_estimate;
  }
};

}