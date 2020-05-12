#pragma once

#include <optional>
#include <vector>
#include <ros/ros.h>

#include "racer/astar/discretized/discretization.h"
#include "racer/astar/sehs/discretization.h"

#include "racer/vehicle/action.h"
#include "racer/vehicle/base_model.h"

#include "racer_ros/discretized_planner.h"

using State = racer::vehicle::kinematic::state;
using DiscreteState = racer::astar::sehs::discrete_state;

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
      return racer::astar::sehs::discretization::from(
          last_known_state.cfg(), occupancy_grid_, next_waypoints_,
          model->chassis->radius(), model->chassis->motor->max_rpm());
  }

};

} // namespace racer_ros
