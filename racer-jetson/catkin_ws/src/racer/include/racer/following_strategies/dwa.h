#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "racer/circuit.h"
#include "racer/following_strategies/following_strategy.h"
#include "racer/following_strategies/target_locator.h"
#include "racer/vehicle_model/motor_model.h"

namespace racer::following_strategies {
template <typename State> class target_error_calculator {
public:
  target_error_calculator()
      : position_error_weight_{0}, heading_error_weight_{0},
        motor_rpm_error_weight_{0}, obstacle_proximity_error_weight_{0} {}

  target_error_calculator(double position_error_weight,
                          double heading_error_weight,
                          double motor_rpm_error_weight,
                          double obstacle_proximity_error_weight,
                          racer::vehicle_model::rpm max_rpm)
      : position_error_weight_(position_error_weight),
        heading_error_weight_(heading_error_weight),
        motor_rpm_error_weight_(motor_rpm_error_weight),
        obstacle_proximity_error_weight_(obstacle_proximity_error_weight),
        max_rpm_(max_rpm) {}

  target_error_calculator(target_error_calculator<State> &&other) = default;
  target_error_calculator<State> &
  operator=(target_error_calculator<State> &&other) = default;

  target_error_calculator(const target_error_calculator<State> &other) =
      default;
  target_error_calculator<State> &
  operator=(const target_error_calculator<State> &other) = default;

  double
  calculate_error(const std::vector<State> &prediction,
                  const racer::trajectory<State> &reference,
                  const std::shared_ptr<racer::occupancy_grid> map) const {
    double total_error = 0;
    auto it_a{prediction.begin()};
    auto it_b{reference.steps().begin()};

    std::size_t steps = 0;

    for (; it_a != prediction.end() && it_b != reference.steps().end();
         ++it_a, ++it_b) {
      const double err = calculate_error(*it_a, it_b->state(), map);
      const double discount =
          (prediction.size() - ++steps) / double(prediction.size());
      total_error += discount * err;
    }

    return total_error / double(prediction.size());
  }

  double
  calculate_error(const State a, const State b,
                  const std::shared_ptr<racer::occupancy_grid> map) const {
    return position_error_weight_ * position_error(a, b) +
           heading_error_weight_ * heading_error(a, b) +
           motor_rpm_error_weight_ * motor_rpm_error(a, b) +
           obstacle_proximity_error_weight_ * obstacle_proximity_error(a, map);
  }

private:
  double position_error_weight_, heading_error_weight_, motor_rpm_error_weight_;
  double obstacle_proximity_error_weight_, max_rpm_;

  double position_error(const State &a, const State &reference) const {
    return a.position().distance_sq(reference.position());
  }

  double heading_error(const State &a, const State &reference) const {
    const auto heading_a = a.configuration().heading_angle().to_normal_angle();
    const auto heading_b =
        reference.configuration().heading_angle().to_normal_angle();

    return heading_a.distance_to(heading_b) / M_PI;
  }

  double motor_rpm_error(const State &a, const State &reference) const {
    return std::abs(reference.motor_rpm() - a.motor_rpm()) / max_rpm_;
  }

  double obstacle_proximity_error(
      const State &a, const std::shared_ptr<racer::occupancy_grid> grid) const {
    const double ideal_distance = 2.0;
    const double dist =
        grid->distance_to_closest_obstacle(a.position(), ideal_distance);
    return (ideal_distance - dist) / ideal_distance;
  }
};

template <typename State> class unfolder {
public:
  unfolder(std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model,
           double time_step_s, int steps)
      : model_{model}, time_step_s_{time_step_s}, steps_{steps} {}

public:
  std::vector<State>
  unfold(const State &origin, const racer::action &action,
         const std::shared_ptr<racer::occupancy_grid> grid) const {
    std::vector<State> next_states{};

    next_states.push_back(origin);
    State last_state = origin;

    for (int i = 0; i < steps_; ++i) {
      // the obstacles in the map are inflated so it is sufficient to check
      // just the grid cell which the center of the vehicle lies in
      last_state = model_->predict_next_state(last_state, action, time_step_s_);
      // if (grid->collides(last_state.position()))
      // {
      //   return {};
      // }

      next_states.push_back(last_state);
    }

    return next_states;
  }

private:
  std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
  double time_step_s_;
  int steps_;
};

template <typename State> class dwa : public following_strategy<State> {
public:
  dwa(const std::vector<racer::action> available_actions,
      const unfolder<State> unfolder,
      const target_locator<State> &target_locator,
      const target_error_calculator<State> &target_error_calculator)
      : available_actions_{available_actions}, unfolder_{unfolder},
        target_error_calculator_{target_error_calculator},
        target_locator_{target_locator} {}

  racer::action select_action(
      const State &current_state, const std::size_t passed_waypoints,
      const racer::trajectory<State> &reference_trajectory,
      const std::shared_ptr<racer::occupancy_grid> map) const override {
    racer::action best_so_far{};
    double lowest_error = HUGE_VAL;

    const auto should_follow =
        reference_trajectory.find_reference_subtrajectory(current_state,
                                                          passed_waypoints);
    for (const auto next_action : available_actions_) {
      const auto trajectory = unfolder_.unfold(current_state, next_action, map);
      if (!trajectory.empty()) {
        const auto final_state = trajectory.back();
        const double error = target_error_calculator_.calculate_error(
            trajectory, should_follow, map);

        if (!best_so_far.is_valid() || error < lowest_error ||
            (error == lowest_error &&
             std::abs(next_action.target_steering_angle()) <
                 std::abs(best_so_far.target_steering_angle()))) {
          lowest_error = error;
          best_so_far = next_action;
        }
      }
    }

    return best_so_far;
  }

private:
  const target_locator<State> target_locator_;
  const unfolder<State> unfolder_;
  std::vector<action> available_actions_;
  target_error_calculator<State> target_error_calculator_;
};

} // namespace racer::following_strategies
