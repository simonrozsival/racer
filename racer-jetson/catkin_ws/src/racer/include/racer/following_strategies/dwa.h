#ifndef DWA_H_
#define DWA_H_

#include <iostream>
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "racer/circuit.h"
#include "racer/following_strategies/following_strategy.h"

namespace racer::following_strategies
{

template <typename TState>
class trajectory_error_calculator
{
public:
    trajectory_error_calculator()
        : position_error_weight_{0},
          heading_error_weight_{0},
          velocity_error_weight_{0},
          velocity_undershooting_overshooting_ratio_{0},
          max_position_error_{0},
          obstacle_proximity_error_weight_{0}
    {
    }

    trajectory_error_calculator(
        double position_error_weight,
        double heading_error_weight,
        double velocity_error_weight,
        double velocity_undershooting_overshooting_ratio,
        double obstacle_proximity_error_weight,
        double max_position_error)
        : position_error_weight_(position_error_weight),
          heading_error_weight_(heading_error_weight),
          velocity_error_weight_(velocity_error_weight),
          velocity_undershooting_overshooting_ratio_(velocity_undershooting_overshooting_ratio),
          max_position_error_(max_position_error),
          obstacle_proximity_error_weight_(obstacle_proximity_error_weight)
    {
    }

    trajectory_error_calculator(trajectory_error_calculator &&other) = default;
    trajectory_error_calculator &operator=(trajectory_error_calculator &&other) = default;

    trajectory_error_calculator(const trajectory_error_calculator &other) = default;
    trajectory_error_calculator &operator=(const trajectory_error_calculator &other) = default;

    double calculate_error(
        const std::list<TState> &attempt,
        const trajectory &reference,
        const occupancy_grid &map) const
    {

        double error = 0;

        std::list<TState>::const_iterator first_it = attempt.begin();
        std::list<trajectory_step<TState>>::const_iterator second_it = reference.steps().begin();

        std::size_t step = 0;
        std::size_t steps = std::min(attempt.size(), reference.steps().size());

        const double total_weight = position_error_weight_ + heading_error_weight_ + velocity_error_weight_ + obstacle_proximity_error_weight_;

        for (; first_it != attempt.end() && second_it != reference.steps().end(); ++first_it, ++second_it)
        {
            double step_error = position_error_weight_ * position_error(*first_it, second_it->state()) + heading_error_weight_ * heading_error(*first_it, second_it->state()) + velocity_error_weight_ * velocity_error(*first_it, second_it->state()) + obstacle_proximity_error_weight_ * obstacle_proximity_error(*first_it, map);

            // double weight = pow(double(steps - step++) / double(steps), 2);
            // error += weight * step_error;
            error += step_error;
        }

        return error / (double(steps) * total_weight);
    }

private:
    double position_error_weight_, heading_error_weight_, velocity_error_weight_;
    double velocity_undershooting_overshooting_ratio_, max_position_error_, obstacle_proximity_error_weight_;

    double position_error(const TState &a, const TState &reference) const
    {
        return (a.position() - reference.position()).length() / max_position_error_;
    }

    double heading_error(const TState &a, const TState &reference) const
    {
        double delta = abs(racer::math::angle(a.position().heading_angle()) - racer::math::angle(reference.position().heading_angle()));
        return std::min(delta, 2 * M_PI - delta);
    }

    double velocity_error(const TState &a, const TState &reference) const
    {
        double speed_ratio = reference.speed() == 0
                                 ? (a.speed() == 0 ? 0 : 1)
                                 : a.speed() / reference.speed();

        return speed_ratio <= 1
                   ? 1 - speed_ratio
                   : speed_ratio / velocity_undershooting_overshooting_ratio_;
    }

    double obstacle_proximity_error(const TState &a, const occupancy_grid &grid) const
    {
        const auto value_at = grid.value_at(a.position().x(), a.position().y());
        const double val = double(value_at) / double(grid.max_value());
        return val;
    }
};

template <typename TState>
class dwa : public following_strategy<TState>
{
public:
    dwa(
        int steps,
        const std::list<racer::action> available_actions,
        const std::shared_ptr<racer::vehicle_model::base_model<TState>> model,
        const trajectory_error_calculator<TState> &trajectory_error_calculator)
        : steps_(steps),
          available_actions_(available_actions),
          model_(model),
          trajectory_error_calculator_(trajectory_error_calculator)
    {
    }

    racer::action select_action(
        const TState &current_state,
        const std::size_t passed_waypoints,
        const racer::trajectory &reference_trajectory,
        const racer::occupancy_grid &map) const override
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
                const double error = trajectory_error_calculator_.calculate_error(trajectory, reference_subtrajectory, map);
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

    void reconfigure(const trajectory_error_calculator &error_calculator)
    {
        trajectory_error_calculator_ = error_calculator;
    }

    std::list<TState> unfold(
        const TState &origin,
        const racer::action &action,
        const racer::occupancy_grid &grid) const
    {

        std::list<TState> next_states{};

        next_states.push_back(origin);
        TState last_state = origin;

        for (int i = 0; i < steps_; ++i)
        {
            last_state = model_->predict_next_state(last_state, action);
            if (!last_state.is_valid())
            {
                break;
            }

            // the obstacles in the map are inflated so it is sufficient to check
            // just the grid cell which the center of the vehicle lies in
            if (grid.collides(last_state.position()))
            {
                return {};
            }

            next_states.push_back(last_state);
        }

        return next_states;
    }

private:
    int steps_;
    std::list<action> available_actions_;
    std::shared_ptr<racer::vehicle_model::base_model<TState>> model_;

    trajectory_error_calculator<TState> trajectory_error_calculator_;
};

} // namespace racer::following_strategies

#endif
