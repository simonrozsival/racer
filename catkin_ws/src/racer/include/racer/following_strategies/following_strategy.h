#pragma once

#include <iostream>

#include "racer/track/occupancy_grid.h"
#include "racer/vehicle/action.h"
#include "racer/vehicle/trajectory.h"

namespace racer::following_strategies {

template <typename State> class following_strategy {
public:
  virtual ~following_strategy() = default;

  virtual racer::vehicle::action select_action(
      const State &current_state, const std::size_t passed_waypoints,
      const racer::vehicle::trajectory<State> &reference_trajectory,
      const std::shared_ptr<racer::track::occupancy_grid> map) const = 0;
};

} // namespace racer::following_strategies
