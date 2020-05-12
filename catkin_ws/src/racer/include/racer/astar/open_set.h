#pragma once

#include <iostream>
#include <queue>
#include <vector>

#include "racer/vehicle/action.h"
#include "racer/vehicle/trajectory.h"

#include "racer/astar/search_node.h"
#include "racer/astar/search_node_comparator.h"

namespace racer::astar
{

template <typename Key, typename State>
class open_set
{
private:
  std::priority_queue<std::shared_ptr<search_node<Key, State>>, std::vector<std::shared_ptr<search_node<Key, State>>>,
                      search_node_comparator<Key, State>>
      data_;
  std::size_t number_of_opened_nodes_;

public:
  open_set(std::unique_ptr<search_node<Key, State>> initial_node) : number_of_opened_nodes_{0}
  {
    push(std::move(initial_node));
  }

  bool is_empty() const
  {
    return data_.size() == 0;
  }

  void push(std::unique_ptr<search_node<Key, State>> node)
  {
    ++number_of_opened_nodes_;
    data_.push(std::move(node));
  }

  std::shared_ptr<search_node<Key, State>> dequeue()
  {
    auto node = data_.top();
    data_.pop();
    return node;
  }

  std::size_t number_of_opened_nodes_since_initial_state() const
  {
    return number_of_opened_nodes_;
  }
};


} // namespace racer::astar
