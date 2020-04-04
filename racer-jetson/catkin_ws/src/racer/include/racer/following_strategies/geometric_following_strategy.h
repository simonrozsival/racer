#pragma once

#include <iostream>
#include <algorithm>

#include "racer/action.h"
#include "racer/trajectory.h"

#include "racer/following_strategies/following_strategy.h"
#include "racer/following_strategies/pid.h"
#include "racer/following_strategies/pure_pursuit.h"

namespace racer::following_strategies
{

template <typename State>
class geometric_following_strategy : public following_strategy<State>
{
public:
    geometric_following_strategy(double max_steering_angle, std::shared_ptr<pid> pid, const pure_pursuit<State> &pursuit)
        : max_steering_angle_(max_steering_angle), pid_(pid), pure_pursuit_(pursuit)
    {
    }

    racer::action select_action(
        const State &current_state,
        const std::size_t passed_waypoints,
        const racer::trajectory<State> &reference_trajectory,
        const std::shared_ptr<racer::occupancy_grid> grid) const override
    {
        auto sub_trajectory = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);

        if (sub_trajectory.empty())
        {
            return {};
        }


        double velocity_command = clamp(pid_->predict_next(current_state.motor_rpm(), sub_trajectory.steps().front().state().motor_rpm()));

        const auto target = pure_pursuit_.find_target(current_state, passed_waypoints, reference_trajectory);
        double steering_command = clamp(pure_pursuit_.select_steering_angle(current_state, target) / max_steering_angle_);

        return {velocity_command, steering_command};
    }

    void reset() override
    {
        pid_.reset();
    }

private:
    std::shared_ptr<pid> pid_;
    pure_pursuit<State> pure_pursuit_;
    const double max_steering_angle_;

    inline double clamp(double value) const
    {
        return std::min(std::max(value, -1.0), 1.0);
    }
};

} // namespace racer::following_strategies
