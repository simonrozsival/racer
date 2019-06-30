#include "Planner.h"

#include <vector>
#include <tf/transform_datatypes.h>

#include "racing-cpp/math/primitives.h"
#include "racing-cpp/racing/circuit.h"
#include "racing-cpp/astar/astar.h"
#include "racing-cpp/astar/hybrid_astar.h"

#include "utils.h"

using namespace racing::kinematic_model;

bool Planner::is_initialized() const {
  return collision_detector_ != nullptr;
}

void Planner::initialize(const nav_msgs::OccupancyGrid& map) {
  collision_detector_ = std::move(racing::collision_detector::precalculate(18, model_, map.info.resolution));
}

racer::TrajectoryMsg Planner::plan(
  const nav_msgs::OccupancyGrid& map,
  const nav_msgs::Odometry& odom,
  const racer::WaypointsMsg& waypoints) const {

  auto initial_state = std::move(msg_to_state(odom));
  auto discrete_initial_state = discretization_.discretize(*initial_state);
  auto grid = std::move(msg_to_grid(map));

  const double initial_heading_angle = initial_state->position.heading_angle;

  std::vector<math::point> waypoints_list;
  double waypoint_radius = waypoints.waypoints[0].radius;
  for (const auto& wp : waypoints.waypoints) {
    waypoints_list.emplace_back(wp.position.x, wp.position.y);
  }

  racing::circuit circuit(
    map.info.resolution,
    initial_state->position,
    waypoints_list,
    waypoint_radius,
    *grid,
    *collision_detector_
  );

  auto problem = std::make_unique<astar::discretized_search_problem<discrete_state>>(
    std::move(initial_state),
    std::move(discrete_initial_state),
    time_step_s_,
    model_,
    available_actions_,
    discretization_,
    circuit
  );

  const auto solution = astar::search<discrete_state, state, trajectory>(
    std::move(problem),
    maximum_number_of_expanded_nodes_
  );

  racer::TrajectoryMsg trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = map.header.frame_id;

  double prev_heading_angle = initial_heading_angle;
  for (const auto& step : solution.steps) {
    racer::TrajectoryStateMsg state;
    
    state.pose.position.x = step.step.position.x;
    state.pose.position.y = step.step.position.y;
    state.pose.position.z = 0;
    
    state.pose.orientation = tf::createQuaternionMsgFromYaw(step.step.position.heading_angle);
    
    state.velocity.linear.x = cos(step.step.position.heading_angle) * step.step.speed;
    state.velocity.linear.y = sin(step.step.position.heading_angle) * step.step.speed;

    state.velocity.angular.z = (prev_heading_angle - step.step.position.heading_angle) / time_step_s_;
    
    trajectory.trajectory.push_back(state);

    prev_heading_angle = step.step.position.heading_angle;
  }

  return trajectory;
}
