#pragma once

#include <iostream>

#include "racer/astar/search_node.h"

namespace racer::astar
{

template <typename Key, typename State>
struct search_node_comparator
{
  bool operator()(const std::shared_ptr<search_node<Key, State>> &a,
                  const std::shared_ptr<search_node<Key, State>> &b) const
  {
    return a->cost_estimate > b->cost_estimate;
  }
};

} // namespace racer::astar
