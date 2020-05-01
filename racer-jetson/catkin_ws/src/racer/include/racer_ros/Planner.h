#pragma once

#include <optional>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"
#include <tf2/LinearMath/Quaternion.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "racer_ros/utils.h"

#include "racer/action.h"
#include "racer/math.h"
#include "racer/track/collision_detection.h"
#include "racer/trajectory.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"

#include "racer/astar/discretized_search_problem.h"
#include "racer/astar/sehs.h"

namespace racer_ros {

struct planning_result {
  std::optional<racer_msgs::Trajectory> trajectory;
  visualization_msgs::MarkerArray visualization;
};

template <typename State, typename DiscreteState> class Planner {
public:
  Planner(std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model,
          std::unique_ptr<racer::astar::discretization<DiscreteState, State>>
              discretization,
          const double time_step_s, const std::string map_frame_id)
      : model_(model), discretization_(std::move(discretization)),
        time_step_s_(time_step_s), map_frame_(map_frame_id) {}

  planning_result plan(const State &initial_state,
                       const std::vector<racer::action> &available_actions,
                       const std::shared_ptr<racer::circuit> circuit,
                       const std::shared_ptr<racer::track::collision_detection>
                           collision_detector,
                       const int next_waypoint) const {
    auto problem = std::make_unique<
        racer::astar::discretized_search_problem<DiscreteState, State>>(
        initial_state, time_step_s_, available_actions, discretization_, model_,
        circuit, collision_detector);

    std::atomic<bool> terminate = false;
    const auto result = racer::astar::search<DiscreteState, State>(
        std::move(problem), terminate);

    visualization_msgs::MarkerArray markers;
    for (std::size_t i = 0; i < result.expanded_nodes.size(); ++i) {
      const auto state = result.expanded_nodes[i];

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();

      marker.ns = "waypoints";
      marker.id = i;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(0.2);

      marker.pose.position.x = state.position().x();
      marker.pose.position.y = state.position().y();
      marker.pose.position.z = 0;

      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, state.configuration().heading_angle());

      marker.pose.orientation.x = myQuaternion.x();
      marker.pose.orientation.y = myQuaternion.y();
      marker.pose.orientation.z = myQuaternion.z();
      marker.pose.orientation.w = myQuaternion.w();

      marker.scale.x = 0.32;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.1;

      markers.markers.push_back(marker);
    }

    if (!result.was_successful()) {
      return {{}, markers};
    }

    racer_msgs::Trajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = map_frame_;

    const auto final_trajectory = result.found_trajectory;

    for (const auto &step : final_trajectory.steps()) {
      racer_msgs::TrajectoryState state;

      // the plan only considers the list of waypoints passed to the planner
      // - the first waypoint will have index 0, so its index has to be offset
      // by the actual index of the first waypoint
      state.next_waypoint.data = next_waypoint + step.passed_waypoints();

      state.pose.position.x = step.state().position().x();
      state.pose.position.y = step.state().position().y();
      state.pose.position.z = 0;

      state.pose.orientation = tf::createQuaternionMsgFromYaw(
          step.state().configuration().heading_angle());

      state.motor_rpm.data = step.state().motor_rpm();

      trajectory.trajectory.push_back(state);
    }

    return {trajectory, markers};
  }

private:
  const std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
  const double time_step_s_;
  const std::shared_ptr<racer::astar::discretization<DiscreteState, State>>
      discretization_;
  const std::string map_frame_;
};

} // namespace racer_ros
