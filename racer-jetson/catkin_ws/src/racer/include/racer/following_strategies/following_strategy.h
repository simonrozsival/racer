#ifndef FOLLOWING_STRATEGY_H_
#define FOLLOWING_STRATEGY_H_

#include <iostream>

#include "racer/action.h"
#include "racer/trajectory.h"
#include "racer/occupancy_grid.h"

namespace racer::following_strategies
{

template <typename TState>
class following_strategy
{
public:
    virtual racer::action select_action(
        const TState &current_state,
        const std::size_t passed_waypoints,
        const racer::trajectory<TState> &reference_trajectory,
        const racer::occupancy_grid &map) const = 0;

    virtual void reset() = 0;
};

} // namespace racer::following_strategies

#endif