#pragma once

#include <iostream>
#include <unordered_set>

#include "racer/astar/search_node.h"

namespace racer::astar
{

template <typename Key, typename State>
class closed_set
{
private:
  std::unordered_set<std::pair<Key, size_t>> data_;

public:
  bool contains(const Key &key, const size_t passed_waypoints) const
  {
    return data_.find(std::pair<Key, size_t>(key, passed_waypoints)) != data_.end();
  }

  bool contains(const search_node<Key, State> &node) const
  {
    return contains(node.key, node.passed_waypoints);
  }

  void add(const search_node<Key, State> &node)
  {
    data_.insert(std::pair<Key, size_t>(node.key, node.passed_waypoints));
  }

  std::size_t size()
  {
    return data_.size();
  }
};

} // namespace racer::astar
