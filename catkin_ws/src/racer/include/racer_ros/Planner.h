#pragma once

#include <optional>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racer_ros/utils.h"

#include "racer/vehicle/action.h"
#include "racer/math.h"
#include "racer/track/collision_detection.h"
#include "racer/vehicle/trajectory.h"
#include "racer/vehicle/configuration.h"
#include "racer/vehicle/kinematic/model.h"

#include "racer/astar/discretized_search_problem.h"
#include "racer/astar/sehs.h"

namespace racer_ros {

template <typename State> class BasePlanner {
public:
  virtual std::optional<racer_msgs::Trajectory>
  plan(const State &initial_state,
       const std::vector<racer::vehicle::action> &available_actions,
       const std::shared_ptr<racer::track::circuit> circuit,
       const std::shared_ptr<racer::track::collision_detection>
           collision_detector,
       const int next_waypoint) const = 0;
};

template <typename State, typename DiscreteState>
class Planner : public BasePlanner<State> {
public:
  Planner(std::shared_ptr<racer::vehicle::model<State>> model,
          std::unique_ptr<racer::astar::discretization<DiscreteState, State>>
              discretization,
          const double time_step_s)
      : model_(model), discretization_(std::move(discretization)),
        time_step_s_(time_step_s) {}

  virtual std::optional<racer_msgs::Trajectory>
  plan(const State &initial_state,
       const std::vector<racer::vehicle::action> &available_actions,
       const std::shared_ptr<racer::track::circuit> circuit,
       const std::shared_ptr<racer::track::collision_detection>
           collision_detector,
       const int next_waypoint) const override {

    auto problem = std::make_unique<
        racer::astar::discretized_search_problem<DiscreteState, State>>(
        initial_state, time_step_s_, available_actions, discretization_, model_,
        circuit, collision_detector);

    std::atomic<bool> terminate = false;
    const auto result = racer::astar::search<DiscreteState, State>(
        std::move(problem), terminate);

    if (!result.was_successful()) {
      return {};
    }

    return racer_ros::trajectory_to_msg(result.found_trajectory, next_waypoint);
  }

private:
  const std::shared_ptr<racer::vehicle::model<State>> model_;
  const double time_step_s_;
  const std::shared_ptr<racer::astar::discretization<DiscreteState, State>>
      discretization_;
};

} // namespace racer_ros
