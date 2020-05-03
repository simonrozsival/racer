#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/following_strategies/following_strategy.h"
#include "racer/following_strategies/target_error_calculator.h"
#include "racer/following_strategies/unfolder.h"
#include "racer/vehicle_model/base_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"

namespace racer::following_strategies
{
  template <typename State>
  class dwa_strategy : public following_strategy<State>
  {
  public:
    dwa_strategy(const std::vector<racer::action> available_actions, const unfolder<State> unfolder,
                 const target_error_calculator<State> &target_error_calculator, double acceleration_weight,
                 const std::size_t lookahead)
        : available_actions_{available_actions}, unfolder_{unfolder}, target_error_calculator_{target_error_calculator}, acceleration_weight_{acceleration_weight}, lookahead_{lookahead}
    {
    }

    racer::action select_action(const State &current_state, const std::size_t passed_waypoints,
                                const racer::trajectory<State> &reference_trajectory,
                                const std::shared_ptr<racer::occupancy_grid> map) const override
    {
      return select_action(current_state, passed_waypoints, reference_trajectory, map, false);
    }

  private:
    const unfolder<State> unfolder_;
    std::vector<action> available_actions_;
    target_error_calculator<State> target_error_calculator_;
    double acceleration_weight_;
    std::size_t lookahead_;

    racer::action select_action(const State &current_state, const std::size_t passed_waypoints,
                                const racer::trajectory<State> &reference_trajectory,
                                const std::shared_ptr<racer::occupancy_grid> map, bool unsafe) const
    {
      racer::action best_so_far{};
      double lowest_error = HUGE_VAL;

      const auto should_follow = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
      for (const auto next_action : available_actions_)
      {
        const auto trajectory = !unsafe ? unfolder_.unfold(current_state, next_action, map, lookahead_) : unfolder_.unfold_unsafe(current_state, next_action, lookahead_);
        if (!trajectory.empty())
        {
          const double error = target_error_calculator_.calculate_error(trajectory, should_follow, map) +
                               acceleration_weight_ * (1 - next_action.throttle());

          if (error < lowest_error)
          {
            lowest_error = error;
            best_so_far = next_action;
          }
        }
      }

      if (!best_so_far.is_valid() && !unsafe)
      {
        // the car is probably too close to some obstacle, there's no other way
        // around it...
        return select_action(current_state, passed_waypoints, reference_trajectory, map, true);
      }

      return best_so_far;
    }
  };

} // namespace racer::following_strategies
