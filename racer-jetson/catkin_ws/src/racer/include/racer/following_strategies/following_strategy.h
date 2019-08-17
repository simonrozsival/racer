#ifndef FOLLOWING_STRATEGY_H_
#define FOLLOWING_STRATEGY_H_

#include <iostream>

#include "racer/vehicle_model/kinematic_bicycle_model.h"
#include "racer/occupancy_grid.h"

using namespace racer::vehicle_model::kinematic_bicycle_model;

namespace racer::following_strategies {

    class following_strategy {
    public:
        virtual std::unique_ptr<action> select_action(
            const state& current_state,
            const std::size_t passed_waypoints,
            const trajectory& reference_trajectory,
            const racer::occupancy_grid& map) const = 0;

        virtual void reset() = 0;
    };

}

#endif