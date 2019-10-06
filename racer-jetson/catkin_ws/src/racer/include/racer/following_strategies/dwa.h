#ifndef DWA_H_
#define DWA_H_

#include <iostream>
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "racer/circuit.h"
#include "racer/vehicle_model/kinematic_bicycle_model.h"

#include "./following_strategy.h"

using namespace racer::vehicle_model::kinematic_bicycle_model;

namespace racer::following_strategies {

    class trajectory_error_calculator {
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

        trajectory_error_calculator(trajectory_error_calculator&& other) = default;
        trajectory_error_calculator& operator=(trajectory_error_calculator&& other) = default;

        trajectory_error_calculator(const trajectory_error_calculator& other) = default;
        trajectory_error_calculator& operator=(const trajectory_error_calculator& other) = default;

        double calculate_error(
            const std::list<state>& attempt,
            const trajectory& reference,
            const occupancy_grid& map) const {

            double error = 0;

            std::list<state>::const_iterator first_it = attempt.begin();
            std::list<trajectory_step>::const_iterator second_it = reference.steps().begin();

            std::size_t step = 0;
            std::size_t steps = std::min(attempt.size(), reference.steps().size());

            const double total_weight = position_error_weight_ + heading_error_weight_ + velocity_error_weight_ + obstacle_proximity_error_weight_;

            for (; first_it != attempt.end() && second_it != reference.steps().end(); ++first_it, ++second_it) {
                double step_error = position_error_weight_ * position_error(*first_it, second_it->step())
                    + heading_error_weight_ * heading_error(*first_it, second_it->step())
                    + velocity_error_weight_ * velocity_error(*first_it, second_it->step())
                    + obstacle_proximity_error_weight_ * obstacle_proximity_error(*first_it, map);

                // double weight = pow(double(steps - step++) / double(steps), 2);
                // error += weight * step_error;
                error += step_error;
            }

            return error / (double(steps) * total_weight);
        }
    private:
        double position_error_weight_, heading_error_weight_, velocity_error_weight_;
        double velocity_undershooting_overshooting_ratio_, max_position_error_, obstacle_proximity_error_weight_;

        double position_error(const state& a, const state& reference) const {
            return (a.position().location() - reference.position().location()).length() / max_position_error_;
        }

        double heading_error(const state& a, const state& reference) const {
            double delta = abs(racer::math::angle(a.position().heading_angle()) - racer::math::angle(reference.position().heading_angle()));
            return std::min(delta, 2 * M_PI - delta);
        }

        double velocity_error(const state& a, const state& reference) const {
            double speed_ratio = reference.speed() == 0
                ? (a.speed() == 0 ? 0 : 1)
                : a.speed() / reference.speed();

            return speed_ratio <= 1
                ? 1 - speed_ratio
                : speed_ratio / velocity_undershooting_overshooting_ratio_;
        }

        double obstacle_proximity_error(const state& a, const occupancy_grid& grid) const {
            const auto value_at = grid.value_at(a.position().location().x(), a.position().location().y());
            const double val = double(value_at) / double(grid.max_value());
            return val;
        }
    };

    class dwa : public following_strategy {
    public:
        dwa(
            int steps,
            const std::list<action> available_actions,
            const std::shared_ptr<model> model,
            const trajectory_error_calculator& trajectory_error_calculator)
            : steps_(steps),
            available_actions_(available_actions),
            model_(model),
            trajectory_error_calculator_(trajectory_error_calculator)
        {
        }

        action select_action(
            const state& current_state,
            const std::size_t passed_waypoints,
            const trajectory& reference_trajectory,
            const racer::occupancy_grid& map) const override {

            const auto& reference_subtrajectory = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
            if (reference_subtrajectory.empty()) {
                return {};
            }

            action best_so_far{};
            double lowest_error = HUGE_VAL;

            for (const auto next_action : available_actions_) {
                const auto trajectory = unfold(current_state, next_action, map);
                if (!trajectory.empty()) {
                  const double error = trajectory_error_calculator_.calculate_error(trajectory, reference_subtrajectory, map);
                  if (error < lowest_error) {
                      lowest_error = error;
                      best_so_far = next_action;
                  }
                }
            }

            return best_so_far;
        }

        void reset() override { }

        void reconfigure(const trajectory_error_calculator& error_calculator) {
            trajectory_error_calculator_ = error_calculator;
        }

        std::list<state> unfold(
            const state& origin,
            const action& action,
            const racer::occupancy_grid& grid) const {

            std::list<state> next_states{};

            // make a copy of the current state and use it as an iterator
            state current = origin;
            next_states.push_back(current);

            for (int i = 0; i < steps_; ++i) {
                auto prediction = model_->predict(current, action);
                if (!prediction.is_valid()) {
                    break;
                }

                current = std::move(prediction);

                // the obstacles in the map are inflated so it is sufficient to check
                // just the grid cell which the center of the vehicle lies in
                if (grid.collides(current.position().location().x(), current.position().location().y())) {
                   return {};
                }

                next_states.push_back(current);
            }

            return next_states;
        }
    private:
        int steps_;
        std::list<action> available_actions_;
        std::shared_ptr<model> model_;
        
        trajectory_error_calculator trajectory_error_calculator_;
    };

}

#endif
