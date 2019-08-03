#ifndef DWA_H_
#define DWA_H_

#include <iostream>

#include <costmap_2d/cost_values.h>

#include "racing/circuit.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"

#include "./following_strategy.h"

using namespace racing::kinematic_model;

namespace racing {

    class trajectory_error_calculator {
    public:
        trajectory_error_calculator(
            double position_error_weight,
            double heading_error_weight,
            double velocity_error_weight,
            double velocity_undershooting_overshooting_ratio,
            double distance_to_obstacles_weight,
            double max_position_error)
            : position_error_weight_(position_error_weight),
            heading_error_weight_(heading_error_weight),
            velocity_error_weight_(velocity_error_weight),
            velocity_undershooting_overshooting_ratio_(velocity_undershooting_overshooting_ratio),
            max_position_error_(max_position_error),
            distance_to_obstacles_weight_(distance_to_obstacles_weight)
        {
        }

        double score(
            const std::unique_ptr<std::list<state>>& attempt,
            const std::unique_ptr<trajectory>& reference,
            const occupancy_grid& costmap) const {

            double score = 0;

            std::list<state>::const_iterator first_it = attempt->begin();
            std::list<trajectory_step>::const_iterator second_it = reference->steps.begin();

            std::size_t step = 0;
            std::size_t steps = std::min(attempt->size(), reference->steps.size());

            for (; first_it != attempt->end() && second_it != reference->steps.end(); ++first_it, ++second_it) {
                double error = position_error_weight_ * position_error(*first_it, second_it->step)
                    + heading_error_weight_ * heading_error(*first_it, second_it->step)
                    + velocity_error_weight_ * velocity_error(*first_it, second_it->step)
                    + distance_to_obstacles_weight_ * distance_to_obstacles(*first_it, costmap);

                double weight = pow(double(steps - step++) / double(steps), 2);

                score += weight * error;
            }

            return score;
        }
    private:
        const double position_error_weight_, heading_error_weight_, velocity_error_weight_;
        const double velocity_undershooting_overshooting_ratio_, max_position_error_, distance_to_obstacles_weight_;

        double position_error(const state& a, const state& reference) const {
            return (a.position.location() - reference.position.location()).length() / max_position_error_;
        }

        double heading_error(const state& a, const state& reference) const {
            double delta = abs(math::angle(a.position.heading_angle) - math::angle(reference.position.heading_angle));
            return std::min(delta, 2 * pi - delta);
        }

        double velocity_error(const state& a, const state& reference) const {
            double speed_ratio = reference.speed == 0
                ? (a.speed == 0 ? 0 : 1)
                : a.speed / reference.speed;

            return speed_ratio <= 1
                ? 1 - speed_ratio
                : speed_ratio / velocity_undershooting_overshooting_ratio_;
        }

        double distance_to_obstacles(const state& a, const occupancy_grid& grid) const {
            return (255.0 - grid.value_at(a.position.location().x, a.position.location().x)) / 255.0;
        }
    };

    class dwa : public following_strategy {
    public:
        dwa(
            int steps,
            const std::list<action> available_actions,
            const std::shared_ptr<model> model,
            std::unique_ptr<trajectory_error_calculator> trajectory_error_calculator)
            : steps_(steps),
            available_actions_(available_actions),
            model_(model),
            trajectory_error_calculator_(std::move(trajectory_error_calculator))
        {
        }

        std::unique_ptr<action> select_action(
            const state& current_state,
            const std::size_t passed_waypoints,
            const trajectory& reference_trajectory,
            const racing::occupancy_grid& costmap) const override {

            const auto& reference_subtrajectory = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
            if (reference_subtrajectory == nullptr) {
                return nullptr;
            }

            std::unique_ptr<action> best_so_far = nullptr;
            double best_score = 1000000000000000000.0;

            for (const auto& next_action : available_actions_) {
                const auto trajectory = unfold(current_state, next_action, costmap);
                if (trajectory) {
                    const double trajectory_score = trajectory_error_calculator_->score(trajectory, reference_subtrajectory, costmap);
                    if (trajectory_score < best_score) {
                        best_score = trajectory_score;
                        best_so_far = std::make_unique<action>(next_action);
                    }
                }
            }

            return std::move(best_so_far);
        }

        void reset() override { }

    private:
        const int steps_;
        const std::list<action> available_actions_;
        const std::shared_ptr<model> model_;
        const std::unique_ptr<trajectory_error_calculator> trajectory_error_calculator_;

        std::unique_ptr <std::list<state>> unfold(
            const state& origin,
            const action& action,
            const racing::occupancy_grid& grid) const {

            std::list<state> next_states;

            // make a copy of the current state and use it as an iterator
            auto current = std::make_unique<state>(origin.position, origin.speed, origin.steering_angle);
            next_states.push_back(*current);

            for (int i = 0; i < steps_; ++i) {
                auto prediction = model_->predict(*current, action);
                if (!prediction) {
                    return nullptr;
                }

                current = std::move(prediction);

                // the costmap is inflated and there is a collision if the center of gravity
                // is in the "deadly" zone
                if (grid.value_at(current->position.x, current->position.y) == costmap_2d::LETHAL_OBSTACLE) {
                    return nullptr;
                }

                next_states.push_back(*current);
            }

            return std::make_unique<std::list<state>>(next_states);
        }
    };

}

#endif
