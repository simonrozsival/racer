#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include <costmap_2d/cost_values.h>

#include "racer/circuit.h"
#include "racer/following_strategies/following_strategy.h"
#include "racer/vehicle_model/motor_model.h"

namespace racer::following_strategies
{
template <typename State>
class trajectory_error_calculator
{
public:
  trajectory_error_calculator()
    : position_error_weight_{ 0 }
    , heading_error_weight_{ 0 }
    , motor_rpm_error_weight_{ 0 }
    , obstacle_proximity_error_weight_{ 0 }
  {
  }

  trajectory_error_calculator(double position_error_weight, double heading_error_weight, double motor_rpm_error_weight,
                              double obstacle_proximity_error_weight, racer::vehicle_model::rpm max_rpm)
    : position_error_weight_(position_error_weight)
    , heading_error_weight_(heading_error_weight)
    , motor_rpm_error_weight_(motor_rpm_error_weight)
    , obstacle_proximity_error_weight_(obstacle_proximity_error_weight)
    , max_rpm_(max_rpm)
  {
  }

  trajectory_error_calculator(trajectory_error_calculator<State> &&other) = default;
  trajectory_error_calculator<State> &operator=(trajectory_error_calculator<State> &&other) = default;

  trajectory_error_calculator(const trajectory_error_calculator<State> &other) = default;
  trajectory_error_calculator<State> &operator=(const trajectory_error_calculator<State> &other) = default;

  double calculate_error(const std::vector<State> &attempt, const racer::trajectory<State> &reference,
                         const std::shared_ptr<racer::occupancy_grid> map) const
  {
    double accumulated_error = 0;

    auto attempt_it = attempt.cbegin();
    auto reference_it = reference.steps().cbegin();

    std::size_t step = 0;
    std::size_t steps = std::min(attempt.size(), reference.steps().size());

    const double total_weight =
        position_error_weight_ + heading_error_weight_ + motor_rpm_error_weight_ + obstacle_proximity_error_weight_;

    for (; attempt_it != attempt.cend() && reference_it != reference.steps().cend(); ++attempt_it, ++reference_it)
    {
      auto at = *attempt_it;
      auto ref = reference_it->state();

      double step_error = position_error_weight_ * position_error(at, ref) +
                          heading_error_weight_ * heading_error(at, ref) +
                          motor_rpm_error_weight_ * motor_rpm_error(at, ref) +
                          obstacle_proximity_error_weight_ * obstacle_proximity_error(at, map);

      // double discount = 1 - pow(double(steps - ++step) / double(steps), 2);
      // step_error *= discount;

      accumulated_error += step_error;
    }

    if (std::isnan(accumulated_error))
    {
      std::cout << "NAN!!" << std::endl;
      return 10000000.0;
    }

    return accumulated_error / (double(steps) * total_weight);
  }

private:
  double position_error_weight_, heading_error_weight_, motor_rpm_error_weight_;
  double obstacle_proximity_error_weight_, max_rpm_;

  double position_error(const State &a, const State &reference) const
  {
    return a.position().distance_sq(reference.position());
  }

  double heading_error(const State &a, const State &reference) const
  {
    const auto heading_a = a.configuration().heading_angle().to_normal_angle();
    const auto heading_b = reference.configuration().heading_angle().to_normal_angle();

    return heading_a.distance_to(heading_b) / M_PI;
  }

  double motor_rpm_error(const State &a, const State &reference) const
  {
    return std::abs(reference.motor_rpm() - a.motor_rpm()) / max_rpm_;
  }

  double obstacle_proximity_error(const State &a, const std::shared_ptr<racer::occupancy_grid> grid) const
  {
    const double ideal_distance = 2.0;
    const double dist = grid->distance_to_closest_obstacle(a.position(), ideal_distance);
    return (ideal_distance - dist) / ideal_distance;
  }
};

template <typename State>
class unfolder
{
public:
  unfolder(std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model, double time_step_s, int steps)
    : model_{ model }, time_step_s_{ time_step_s }, steps_{ steps }
  {
  }

public:
  std::vector<State> unfold(const State &origin, const racer::action &action,
                            const std::shared_ptr<racer::occupancy_grid> grid) const
  {
    std::vector<State> next_states{};

    next_states.push_back(origin);
    State last_state = origin;

    for (int i = 0; i < steps_; ++i)
    {
      // the obstacles in the map are inflated so it is sufficient to check
      // just the grid cell which the center of the vehicle lies in
      last_state = model_->predict_next_state(last_state, action, time_step_s_);
      if (grid->collides(last_state.position()))
      {
        return {};
      }

      next_states.push_back(last_state);
    }

    return next_states;
  }

private:
  std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
  double time_step_s_;
  int steps_;
};

template <typename State>
class dwa : public following_strategy<State>
{
public:
  dwa(const std::vector<racer::action> available_actions, const unfolder<State> unfolder,
      const trajectory_error_calculator<State> &trajectory_error_calculator)
    : available_actions_{ available_actions }
    , unfolder_{ unfolder }
    , trajectory_error_calculator_{ trajectory_error_calculator }
  {
  }

  racer::action select_action(const State &current_state, const std::size_t passed_waypoints,
                              const racer::trajectory<State> &reference_trajectory,
                              const std::shared_ptr<racer::occupancy_grid> map) const override
  {
    const auto &reference_subtrajectory =
        reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
    if (reference_subtrajectory.empty())
    {
      std::cout << "empty reference subtrajectory" << std::endl;
      return {};
    }

    racer::action best_so_far{};
    double lowest_error = HUGE_VAL;

    for (const auto next_action : available_actions_)
    {
      const auto trajectory = unfolder_.unfold(current_state, next_action, map);
      if (!trajectory.empty())
      {
        const double error = trajectory_error_calculator_.calculate_error(trajectory, reference_subtrajectory, map);

        if (!best_so_far.is_valid() || error < lowest_error ||
            (error == lowest_error &&
             std::abs(next_action.target_steering_angle()) < std::abs(best_so_far.target_steering_angle())))
        {
          lowest_error = error;
          best_so_far = next_action;
        }
      }
    }

    return best_so_far;
  }

private:
  const unfolder<State> unfolder_;
  std::vector<action> available_actions_;
  trajectory_error_calculator<State> trajectory_error_calculator_;
};

}  // namespace racer::following_strategies
