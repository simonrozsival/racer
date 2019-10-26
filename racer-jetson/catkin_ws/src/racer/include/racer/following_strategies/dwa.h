#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "racer/circuit.h"
#include "racer/following_strategies/following_strategy.h"

namespace racer::following_strategies
{

template <typename State>
class trajectory_error_calculator
{
public:
    trajectory_error_calculator()
        : position_error_weight_{0},
          heading_error_weight_{0},
          motor_rpm_error_weight_{0},
          max_position_error_{0},
          obstacle_proximity_error_weight_{0}
    {
    }

    trajectory_error_calculator(
        double position_error_weight,
        double heading_error_weight,
        double motor_rpm_error_weight,
        double obstacle_proximity_error_weight,
        double max_position_error)
        : position_error_weight_(position_error_weight),
          heading_error_weight_(heading_error_weight),
          motor_rpm_error_weight_(motor_rpm_error_weight),
          max_position_error_(max_position_error),
          obstacle_proximity_error_weight_(obstacle_proximity_error_weight)
    {
    }

    trajectory_error_calculator(trajectory_error_calculator<State> &&other) = default;
    trajectory_error_calculator<State> &operator=(trajectory_error_calculator<State> &&other) = default;

    trajectory_error_calculator(const trajectory_error_calculator<State> &other) = default;
    trajectory_error_calculator<State> &operator=(const trajectory_error_calculator<State> &other) = default;

    double calculate_error(
        const std::vector<State> &attempt,
        const racer::trajectory<State> &reference,
        const std::shared_ptr<racer::occupancy_grid> map) const
    {
        double error = 0;

        auto first_it = attempt.cbegin();
        auto second_it = reference.steps().cbegin();

        std::size_t step = 0;
        std::size_t steps = std::min(attempt.size(), reference.steps().size());

        const double total_weight = position_error_weight_ + heading_error_weight_ + motor_rpm_error_weight_ + obstacle_proximity_error_weight_;

        for (; first_it != attempt.cend() && second_it != reference.steps().cend(); ++first_it, ++second_it)
        {
            double step_error = position_error_weight_ * position_error(*first_it, second_it->state()) + heading_error_weight_ * heading_error(*first_it, second_it->state()) + motor_rpm_error_weight_ * motor_rpm_error(*first_it, second_it->state()) + obstacle_proximity_error_weight_ * obstacle_proximity_error(*first_it, map);

            // double weight = pow(double(steps - step++) / double(steps), 2);
            // error += weight * step_error;
            error += step_error;
        }

        return error / (double(steps) * total_weight);
    }

private:
    double position_error_weight_, heading_error_weight_, motor_rpm_error_weight_;
    double max_position_error_, obstacle_proximity_error_weight_;

    double position_error(const State &a, const State &reference) const
    {
        return (a.position() - reference.position()).length() / max_position_error_;
    }

    double heading_error(const State &a, const State &reference) const
    {
        return a.configuration().heading_angle().distance_to(reference.configuration().heading_angle()) / (2 * M_PI);
    }

    double motor_rpm_error(const State &a, const State &reference) const
    {
        double speed_ratio = reference.motor_rpm() == 0.0
                                 ? (a.motor_rpm() == 0.0 ? 0.0 : 1.0)
                                 : a.motor_rpm() / reference.motor_rpm();

        return std::max(0.0, std::abs(1.0 - speed_ratio));
    }

    double obstacle_proximity_error(const State &a, const std::shared_ptr<racer::occupancy_grid> grid) const
    {
        const auto value_at = grid->value_at(a.position().x(), a.position().y());
        const double val = double(value_at) / double(grid->max_value());
        return val;
    }
};

template <typename State>
class dwa : public following_strategy<State>
{
public:
    dwa(
        const int steps,
        const std::vector<racer::action> available_actions,
        const std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model,
        const trajectory_error_calculator<State> &trajectory_error_calculator,
        const double time_step_s_)
        : steps_{steps},
          available_actions_{available_actions},
          model_{model},
          trajectory_error_calculator_{trajectory_error_calculator},
          time_step_s_{time_step_s_}
    {
    }

    racer::action select_action(
        const State &current_state,
        const std::size_t passed_waypoints,
        const racer::trajectory<State> &reference_trajectory,
        const std::shared_ptr<racer::occupancy_grid> map) const override
    {

        const auto &reference_subtrajectory = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
        if (reference_subtrajectory.empty())
        {
            return {};
        }

        racer::action best_so_far{};
        double lowest_error = HUGE_VAL;

        for (const auto next_action : available_actions_)
        {
            const auto trajectory = unfold(current_state, next_action, map);
            if (!trajectory.empty())
            {
                const double error = trajectory_error_calculator_.calculate_error(
                    trajectory, reference_subtrajectory, map);
                if (error < lowest_error)
                {
                    lowest_error = error;
                    best_so_far = next_action;
                }
            }
        }

        return best_so_far;
    }

    void reset() override {}

    void reconfigure(const trajectory_error_calculator<State> &error_calculator)
    {
        trajectory_error_calculator_ = error_calculator;
    }

    std::vector<State> unfold(
        const State &origin,
        const racer::action &action,
        const std::shared_ptr<racer::occupancy_grid> grid) const
    {

        std::vector<State> next_states{};

        next_states.push_back(origin);
        State last_state = origin;

        for (int i = 0; i < steps_; ++i)
        {
            last_state = model_->predict_next_state(last_state, action, time_step_s_);
            if (!last_state.is_valid())
            {
                break;
            }

            // the obstacles in the map are inflated so it is sufficient to check
            // just the grid cell which the center of the vehicle lies in
            if (grid->collides(last_state.position()))
            {
                return {};
            }

            next_states.push_back(last_state);
        }

        return next_states;
    }

private:
    int steps_;
    std::vector<action> available_actions_;
    std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
    trajectory_error_calculator<State> trajectory_error_calculator_;
    double time_step_s_;
};

} // namespace racer::following_strategies
