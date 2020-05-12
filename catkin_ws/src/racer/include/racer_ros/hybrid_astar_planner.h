#pragma once

#include <optional>
#include <vector>
#include <ros/ros.h>

#include "racer/astar/discretized/discretization.h"
#include "racer/astar/hybrid_astar/discretization.h"

#include "racer/vehicle/action.h"
#include "racer/vehicle/base_model.h"

#include "racer_ros/discretized_planner.h"
#include "racer_ros/utils.h"

using State = racer::vehicle::kinematic::state;
using DiscreteState = racer::astar::hybrid_astar::discrete_state;

namespace racer_ros {

class hybrid_astar_planner : public discretized_planner<DiscreteState> {
public:
  hybrid_astar_planner(
    std::shared_ptr<racer::vehicle::base_vehicle_model<State>> model,
    const std::vector<racer::vehicle::action> available_actions,
    const double time_step_s,
    const double safety_margin,
    ros::Publisher debug_map_pub)
      : discretized_planner<DiscreteState>(model, available_actions, time_step_s, safety_margin, debug_map_pub) {}

protected:
  virtual std::shared_ptr<racer::astar::discretized::base_discretization<DiscreteState, State>> create_discretization() const override {
      auto cell_size = 3 * model_->chassis->radius();
      return std::make_shared<racer::astar::hybrid_astar::discretization>(
              cell_size, cell_size, 2 * M_PI / 18.0,
              model_->chassis->motor->max_rpm() / 20.0);
  }

};

} // namespace racer_ros