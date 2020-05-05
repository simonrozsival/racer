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
class three_stage_dwa_strategy : public following_strategy<State>
{
public:
  three_stage_dwa_strategy(const std::vector<racer::action> available_actions, const unfolder<State> unfolder,
                           const target_error_calculator<State> &target_error_calculator, const std::size_t lookahead)
    : available_actions_{ available_actions }
    , unfolder_{ unfolder }
    , target_error_calculator_{ target_error_calculator }
    , lookahead_{ lookahead }
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
  std::size_t lookahead_;

  racer::action select_action(const State &current_state, const std::size_t passed_waypoints,
                              const racer::trajectory<State> &reference_trajectory,
                              const std::shared_ptr<racer::occupancy_grid> map, bool unsafe) const
  {
    racer::action best_so_far{};
    double lowest_error = HUGE_VAL;

    const auto should_follow = reference_trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
    for (const auto first_action : available_actions_)
    {
      double best_combo = HUGE_VAL;

      for (const auto second_action : available_actions_)
      {
        for (const auto third_action : available_actions_)
        {
          racer::action actions[3] = { first_action, second_action, third_action };
          auto trajectory = unfold(current_state, actions, map, unsafe);
          if (!trajectory.empty())
          {
            const double error = target_error_calculator_.calculate_error(trajectory, should_follow, map);
            if (error < best_combo)
            {
              best_combo = error;
            }
          }
        }
      }

      if (best_combo < lowest_error)
      {
        lowest_error = best_combo;
        best_so_far = first_action;
      }
    }

    if (!best_so_far.is_valid() && !unsafe)
    {
      // the car is probably too close to some obstacle, there's no other way around it...
      return select_action(current_state, passed_waypoints, reference_trajectory, map, true);
    }

    return best_so_far;
  }

  std::vector<State> unfold(const State &current_state, const racer::action actions[3],
                            const std::shared_ptr<racer::occupancy_grid> map, bool unsafe) const
  {
    std::vector<State> trajectory{ current_state };
    trajectory.reserve(lookahead_);
    const std::size_t third_of_lookahead = std::ceil(lookahead_ / 3.0);

    for (std::size_t i{ 0 }; i < 3; ++i)
    {
      auto segment = !unsafe ? unfolder_.unfold(trajectory.back(), actions[i], map, third_of_lookahead) :
                               unfolder_.unfold_unsafe(trajectory.back(), actions[i], third_of_lookahead);
      trajectory.insert(trajectory.end(), segment.begin(), segment.end());
    }

    return trajectory;
  }
};

}  // namespace racer::following_strategies
