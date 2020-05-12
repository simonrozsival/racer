#pragma once

#include <iostream>
#include <vector>

#include "racer/vehicle/action.h"

namespace racer::astar
{

template <typename Key, typename State>
struct search_node
{
  const Key key;
  const std::vector<State> states;
  const racer::vehicle::action previous_action;
  const double cost_to_come;
  const double cost_estimate;
  const std::size_t passed_waypoints;
  const std::weak_ptr<search_node<Key, State>> parent;

  search_node(Key key, std::vector<State> states, racer::vehicle::action previous_action,
              std::weak_ptr<search_node<Key, State>> parent, double cost_to_come, double cost_estimate,
              std::size_t passed_waypoints)
      : key{key}, states{states}, previous_action{previous_action}, cost_to_come{cost_to_come}, cost_estimate{cost_estimate}, passed_waypoints{passed_waypoints}, parent{parent}
  {
  }

  static auto for_initial_state(Key key, State state)
  {
    return std::make_unique<search_node<Key, State>>(key, std::vector<State>{state}, racer::vehicle::action{0, 0},
                                                      std::weak_ptr<search_node<Key, State>>(), 0, 0, 0);
  }

  constexpr double estimated_cost_to_go() const
  {
    return cost_estimate - cost_to_come;
  }

  inline State final_state() const
  {
    return states.back();
  }
};

} // namespace racer::astar
