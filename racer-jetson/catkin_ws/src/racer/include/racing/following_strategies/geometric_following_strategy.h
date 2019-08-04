#ifndef GEOMETRIC_FOLLOWING_STRATEGY_H_
#define GEOMETRIC_FOLLOWING_STRATEGY_H_

#include <iostream>
#include <algorithm>

#include "racing/vehicle_model/kinematic_bicycle_model.h"

#include "./following_strategy.h"
#include "./pid.h"
#include "./pure_pursuit.h"

using namespace racing::kinematic_model;

namespace racing {

    class geometric_following_strategy : public following_strategy {
    public:
        geometric_following_strategy(double max_steering_angle, std::shared_ptr<pid> pid, std::shared_ptr<pure_pursuit> pursuit)
            : max_steering_angle_(max_steering_angle), pid_(pid), pure_pursuit_(pursuit)
        {
        }
        
        std::unique_ptr<action> select_action(
            const state& current_state,
            const std::size_t passed_waypoints,
            const trajectory& reference_trajectory,
            const racing::occupancy_grid& grid) const override {

            auto sub_trajectory = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);

            if (!sub_trajectory || sub_trajectory->steps.size() == 0) {
                return nullptr;
            }

            double velocity_command = clamp(pid_->predict_next(current_state.speed, sub_trajectory->steps.front().step.speed));
            double steering_command = clamp(pure_pursuit_->select_steering_angle(current_state, passed_waypoints, reference_trajectory) / max_steering_angle_);

            return std::make_unique<action>(velocity_command, steering_command);
        }

        void reset() override {
            pid_.reset();
        }

    private:
        std::shared_ptr<pid> pid_;
        std::shared_ptr<pure_pursuit> pure_pursuit_;
        const double max_steering_angle_;

        inline double clamp(double value) const {
            return std::min(std::max(value, -1.0), 1.0);
        }
    };

}

#endif
