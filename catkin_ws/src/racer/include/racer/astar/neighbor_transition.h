#pragma once

#include <atomic>
#include <chrono>
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

#include "racer/vehicle/action.h"
#include "racer/vehicle/trajectory.h"

namespace racer::astar
{

template <typename Key, typename State>
struct neighbor_transition
{
  neighbor_transition(Key key, std::vector<State> states, racer::vehicle::action previous_action, const double cost)
      : key{key}, states{states}, previous_action{previous_action}, cost{cost}
  {
  }

  const Key key;
  const std::vector<State> states;
  const racer::vehicle::action previous_action;

  const double cost;

  const State &final_state() const
  {
    return states.back();
  }
};

} // namespace racer::astar
