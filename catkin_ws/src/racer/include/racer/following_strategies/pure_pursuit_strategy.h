#pragma once

#include <iostream>

#include "racer/following_strategies/pure_pursuit.h"
#include "racer/following_strategies/target_locator.h"
#include "racer/math.h"
#include "racer/track/racing_line.h"
#include "racer/trajectory.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"

namespace racer::following_strategies
{
    template <typename State>
    class pure_pursuit_strategy : public following_strategy<State>
    {
    public:
        pure_pursuit_strategy(
            target_locator<State> locator, pure_pursuit<State> pursuit,
            const std::shared_ptr<racer::vehicle_model::kinematic::model> model)
            : target_locator_{locator}, pure_pursuit_{pursuit}, model_{model} {}

        racer::action select_action(
            const State &current_state, const std::size_t passed_waypoints,
            const racer::trajectory<State> &reference_trajectory,
            const std::shared_ptr<racer::occupancy_grid> map) const override
        {
            const auto closest = reference_trajectory.find_reference_state(
                current_state, passed_waypoints);
            const auto target = target_locator_.find_target(
                current_state, passed_waypoints, reference_trajectory);
            const auto steering_angle = pure_pursuit_.select_steering_angle(
                current_state, target.configuration());

            const auto throttle =
                target.motor_rpm() > current_state.motor_rpm()
                    ? 1.0
                    : std::clamp(target.motor_rpm() / model_->chassis->motor->max_rpm(), -1.0, 1.0);

            const auto steering_input = std::clamp(steering_angle / racer::math::angle::from_degrees(15), -1.0, 1.0);
            //const auto steering_input = model_->chassis->steering_servo->action_input_for_angle(steering_angle);

            return {throttle, steering_input};
        }

    private:
        const pure_pursuit<State> pure_pursuit_;
        const target_locator<State> target_locator_;
        const std::shared_ptr<racer::vehicle_model::kinematic::model> model_;
    };

} // namespace racer::following_strategies
