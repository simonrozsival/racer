#pragma once

#include <optional>
#include <ros/ros.h>
#include <vector>

#include "racer/astar/discretized/base_discretization.h"
#include "racer/astar/sehs/discretization.h"

#include "racer/vehicle/action.h"
#include "racer/vehicle/kinematic/model.h"

#include "racer_ros/discretized_planner.h"

namespace racer_ros {

class sehs_planner
    : public discretized_planner<racer::astar::sehs::discrete_state> {
public:
  sehs_planner(std::shared_ptr<racer::vehicle::kinematic::model> model,
               const std::vector<racer::vehicle::action> available_actions,
               const double time_step_s, const double safety_margin,
               ros::Publisher debug_map_pub)
      : discretized_planner<racer::astar::sehs::discrete_state>(
            model, available_actions, time_step_s, safety_margin,
            debug_map_pub) {}

protected:
  virtual std::shared_ptr<racer::astar::discretized::base_discretization<
      racer::astar::sehs::discrete_state, racer::vehicle::kinematic::state>>
  create_discretization() const override {
    return racer::astar::sehs::discretization::from(
        last_known_state_.cfg(), occupancy_grid_, next_waypoints_,
        model_->chassis->radius(), model_->chassis->motor->max_rpm());
  }
};

} // namespace racer_ros
