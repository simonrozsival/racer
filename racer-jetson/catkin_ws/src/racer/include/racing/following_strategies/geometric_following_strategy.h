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
        geometric_following_strategy(std::unique_ptr<pid> pid, std::unique_ptr<pure_pursuit> pursuit)
            : pid_(std::move(pid), pure_pursuit_(std::move(pursuit))
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

            double velocity_command = std::clamp(pid_->predict_next(current_state.speed, sub_trajectory->steps.front().step.speed), -1.0, 1.0);
            double steering_command = std::clamp(pure_pursuit_->select_steering_angle(current_state, passed_waypoints, reference_trajectory), -1.0, 1.0);

            return std::make_unique<action>(velocity_command, steering_command);
        }

        void reset() override {
            pid_.reset();
        }

    private:
        const std::unique_ptr<pid> pid_;
        const std::unique_ptr<pure_pursuit> pure_pursuit_;
    };

}

#endif