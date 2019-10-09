#include "racer_ros/Planner.h"

#include <vector>
#include <tf/transform_datatypes.h>

#include "racer/circuit.h"
#include "racer/astar/astar.h"

using namespace racer;
using namespace racer::vehicle_model;
using namespace racer::astar::sehs;

namespace racer_ros
{

std::unique_ptr<racer_msgs::Trajectory> Planner::plan(
    const occupancy_grid &grid,
    const kinematic::state &state,
    const std::list<action> &available_actions,
    const std::vector<racer::math::point> &waypoints,
    const int next_waypoint,
    const double waypoint_radius) const
{

  auto discrete_initial_state = discretization_.discretize(state);
  const double initial_heading_angle = state.position().heading_angle();

  racer::circuit circuit(
      state.position(),
      waypoints,
      waypoint_radius,
      grid);

  auto initial_state{state};
  auto problem = std::make_unique<racer::astar::discretized_search_problem<discrete_state>>(
      initial_state,
      discrete_initial_state,
      time_step_s_,
      model_,
      available_actions,
      discretization_,
      circuit);

  const auto result = racer::astar::search<discrete_state, kinematic::state>(
      std::move(problem),
      maximum_number_of_expanded_nodes_);

  if (!result.was_successful())
  {
    return nullptr;
  }

  auto trajectory = std::make_unique<racer_msgs::Trajectory>();
  trajectory->header.stamp = ros::Time::now();
  trajectory->header.frame_id = map_frame_;

  double prev_heading_angle = initial_heading_angle;
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

    state.pose.orientation = tf::createQuaternionMsgFromYaw(step.state().position().heading_angle());

    state.velocity.linear.x = cos(step.state().position().heading_angle()) * step.state().speed();
    state.velocity.linear.y = sin(step.state().position().heading_angle()) * step.state().speed();

    state.velocity.angular.z = (prev_heading_angle - step.state().position().heading_angle()) / time_step_s_;

    trajectory->trajectory.push_back(state);

    prev_heading_angle = step.state().position().heading_angle();
  }

  return trajectory;
}

} // namespace racer_ros