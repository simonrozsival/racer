#ifndef GEOMETRIC_FOLLOWING_STRATEGY_H_
#define GEOMETRIC_FOLLOWING_STRATEGY_H_

#include <iostream>
#include <algorithm>

#include "racer/vehicle_model/kinematic_bicycle_model.h"

#include "./following_strategy.h"
#include "./pid.h"
#include "./pure_pursuit.h"

using namespace racer::vehicle_model::kinematic_bicycle_model;

namespace racer::following_strategies {

    class geometric_following_strategy : public following_strategy {
    public:
        geometric_following_strategy(double max_steering_angle, std::shared_ptr<pid> pid, const pure_pursuit& pursuit)
            : max_steering_angle_(max_steering_angle), pid_(pid), pure_pursuit_(pursuit)
        {
        }
        
        action select_action(
            const state& current_state,
            const std::size_t passed_waypoints,
            const trajectory& reference_trajectory,
            const racer::occupancy_grid& grid) const override {

            auto sub_trajectory = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);

            if (sub_trajectory.empty()) {
                return {};
            }

            double velocity_command = clamp(pid_->predict_next(current_state.speed(), sub_trajectory.steps().front().step().speed()));
            double steering_command = clamp(pure_pursuit_.select_steering_angle(current_state, passed_waypoints, reference_trajectory) / max_steering_angle_);

            return { velocity_command, steering_command };
        }

        void reset() override {
            pid_.reset();
        }

    private:
        std::shared_ptr<pid> pid_;
        pure_pursuit pure_pursuit_;
        const double max_steering_angle_;

        inline double clamp(double value) const {
            return std::min(std::max(value, -1.0), 1.0);
        }
    };

}

#endif
