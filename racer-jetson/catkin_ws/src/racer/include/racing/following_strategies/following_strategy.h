#ifndef FOLLOWING_STRATEGY_H_
#define FOLLOWING_STRATEGY_H_

#include <iostream>

#include "racing/vehicle_model/kinematic_bicycle_model.h"
#include "racing/collision_detection/occupancy_grid.h"

using namespace racing::kinematic_model;

namespace racing {

    class following_strategy {
    public:
        virtual std::unique_ptr<action> select_action(
            const state& current_state,
            const std::size_t passed_waypoints,
            const trajectory& reference_trajectory,
            const racing::occupancy_grid& grid) const = 0;

        virtual void reset() = 0;
    };

}

#endif