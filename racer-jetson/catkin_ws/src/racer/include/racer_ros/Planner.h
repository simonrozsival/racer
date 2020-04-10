#pragma once

#include <vector>
#include <optional>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racer_ros/utils.h"

#include "racer/action.h"
#include "racer/math.h"
#include "racer/track/collision_detection.h"
#include "racer/trajectory.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"

#include "racer/astar/discretized_search_problem.h"
#include "racer/astar/sehs.h"

namespace racer_ros
{
template <typename State, typename DiscreteState>
class Planner
{
public:
  Planner(std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model,
          std::unique_ptr<racer::astar::discretization<DiscreteState, State>> discretization, const double time_step_s,
          const std::string map_frame_id)
      : model_(model), discretization_(std::move(discretization)), time_step_s_(time_step_s), map_frame_(map_frame_id)
  {
  }

  std::optional<racer_msgs::Trajectory>
  plan(const State &initial_state, const std::vector<racer::action> &available_actions,
       const std::shared_ptr<racer::circuit> circuit,
       const std::shared_ptr<racer::track::collision_detection> collision_detector, const int next_waypoint) const
  {
    auto problem = std::make_unique<racer::astar::discretized_search_problem<DiscreteState, State>>(
        initial_state, time_step_s_, available_actions, discretization_, model_, circuit, collision_detector);

    std::atomic<bool> terminate = false;
    const auto result = racer::astar::search<DiscreteState, State>(std::move(problem), terminate);

    if (!result.was_successful())
    {
      return {};
    }

    racer_msgs::Trajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = map_frame_;

    for (const auto &step : result.found_trajectory.steps())
    {
      racer_msgs::TrajectoryState state;

      // the plan only considers the list of waypoints passed to the planner
      // - the first waypoint will have index 0, so its index has to be offset
      // by the actual index of the first waypoint
      state.next_waypoint.data = next_waypoint + step.passed_waypoints();

      state.pose.position.x = step.state().position().x();
      state.pose.position.y = step.state().position().y();
      state.pose.position.z = 0;

      state.pose.orientation = tf::createQuaternionMsgFromYaw(step.state().configuration().heading_angle());

      state.motor_rpm.data = step.state().motor_rpm();

      trajectory.trajectory.push_back(state);
    }

    return trajectory;
  }

private:
  const std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
  const double time_step_s_;
  const std::shared_ptr<racer::astar::discretization<DiscreteState, State>> discretization_;
  const std::string map_frame_;
};

} // namespace racer_ros
