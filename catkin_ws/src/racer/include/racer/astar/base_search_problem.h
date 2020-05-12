#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

#include "racer/vehicle/action.h"
#include "racer/vehicle/trajectory.h"

#include "racer/astar/neighbor_transition.h"
#include "racer/astar/search_node.h"

namespace racer::astar
{

template <typename Key, typename State>
class base_search_problem
{
public:
  virtual ~base_search_problem() = default;
  virtual std::vector<neighbor_transition<Key, State>> valid_neighbors(const search_node<Key, State> &node) const = 0;
  virtual const bool is_goal(std::size_t passed_waypoints) const = 0;
  virtual const bool passes_waypoint(const std::vector<State> &examined_state, size_t passed_waypoints) const = 0;
  virtual const double estimate_cost_to_go(const State &state, std::size_t passed_waypoints) const = 0;
  virtual const racer::vehicle::trajectory<State> reconstruct_trajectory(const search_node<Key, State> &node) const = 0;
  virtual std::unique_ptr<search_node<Key, State>> initial_search_node() const = 0;
};

} // namespace racer::astar
