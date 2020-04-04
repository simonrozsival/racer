#pragma once

#include <iostream>

#include "racer/action.h"
#include "racer/occupancy_grid.h"
#include "racer/trajectory.h"

namespace racer::following_strategies
{
template <typename State>
class following_strategy
{
public:
  virtual ~following_strategy() = default;

  virtual racer::action select_action(const State &current_state, const std::size_t passed_waypoints,
                                      const racer::trajectory<State> &reference_trajectory,
                                      const std::shared_ptr<racer::occupancy_grid> map) const = 0;
};

}  // namespace racer::following_strategies
