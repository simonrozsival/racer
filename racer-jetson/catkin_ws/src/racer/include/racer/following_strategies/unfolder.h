#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/vehicle_model/base_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"

namespace racer::following_strategies {

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
      if (grid->collides(last_state.position())) {
        return {};
      }

      next_states.push_back(last_state);
    }

    return next_states;
  }

  auto unfold_unsafe(const State &origin, const racer::action &action) const {
    std::vector<State> next_states{};

    next_states.push_back(origin);
    State last_state = origin;

    for (int i = 0; i < steps_; ++i) {
      // the obstacles in the map are inflated so it is sufficient to check
      // just the grid cell which the center of the vehicle lies in
      last_state = model_->predict_next_state(last_state, action, time_step_s_);
      next_states.push_back(last_state);
    }

    return next_states;
  }

private:
  std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
  double time_step_s_;
  int steps_;
};

} // namespace racer::following_strategies