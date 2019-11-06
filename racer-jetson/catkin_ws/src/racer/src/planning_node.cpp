#include <iostream>
#include <ros/ros.h>
#include <mutex>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Waypoints.h>
#include <racer_msgs/Trajectory.h>

#include "racer/math.h"
#include "racer/action.h"
#include "racer/trajectory.h"

#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"
#include "racer/sehs/space_exploration.h"
#include "racer/astar/sehs.h"
#include "racer/track_analysis.h"

#include "racer_ros/utils.h"
#include "racer_ros/Planner.h"

std::mutex lock;

using State = racer::vehicle_model::kinematic::state;
using DiscreteState = racer::astar::sehs::kinematic::discrete_state;

State last_known_state;
std::shared_ptr<racer::occupancy_grid> occupancy_grid;
std::shared_ptr<racer::circuit> circuit;

int next_waypoint;
double waypoint_radius;
std::vector<racer::math::point> next_waypoints;

std::string map_frame_id;
std::unique_ptr<racer_ros::Planner<State, DiscreteState>> planner;

std::shared_ptr<racer::vehicle_model::vehicle_chassis> vehicle = racer::vehicle_model::vehicle_chassis::rc_beast();
auto model = std::make_shared<racer::vehicle_model::kinematic::model>(vehicle);

int number_of_expanded_points = 12;
double time_step_s = 1.0 / 25.0;

void state_update(const racer_msgs::State::ConstPtr &state)
{
  std::lock_guard<std::mutex> guard(lock);

  racer::vehicle_configuration position{state->x, state->y, state->heading_angle};
  last_known_state = {position, state->motor_rpm, state->steering_angle};
}

void waypoints_update(const racer_msgs::Waypoints::ConstPtr &waypoints)
{
  std::lock_guard<std::mutex> guard(lock);

  next_waypoints.clear();
  waypoint_radius = waypoints->waypoints[0].radius;
  next_waypoint = waypoints->next_waypoint;

  for (const auto &wp : waypoints->waypoints)
  {
    next_waypoints.emplace_back(wp.position.x, wp.position.y);
  }

  circuit = std::make_shared<racer::circuit>(next_waypoints, waypoint_radius, occupancy_grid);

  // Space Exploration
  racer::sehs::space_exploration exploration{1.0 * vehicle->radius(), 4 * vehicle->radius(), number_of_expanded_points};
  const auto path_of_circles = exploration.explore_grid(occupancy_grid, last_known_state.configuration(), next_waypoints);
  if (path_of_circles.empty())
  {
    ROS_WARN("Space exploration failed, goal is inaccessible.");
    return;
  }

  // Discretization based on Space Exploration
  auto discretization = std::make_unique<racer::astar::sehs::kinematic::discretization>(
      path_of_circles, 24, vehicle->motor->max_rpm() / 10.0);

  // Planner for the next waypoint
  planner = std::make_unique<racer_ros::Planner<State, DiscreteState>>(
      model,
      std::move(discretization),
      time_step_s,
      map_frame_id);
}

std::shared_ptr<racer::occupancy_grid> load_map(ros::NodeHandle &node)
{
  // get the base map for space exploration
  while (!ros::service::waitForService("static_map", ros::Duration(3.0)))
  {
    ROS_INFO("'planning_node': Map service isn't available yet.");
    continue;
  }

  auto map_service_client = node.serviceClient<nav_msgs::GetMap>("/static_map");

  nav_msgs::GetMap::Request map_req;
  nav_msgs::GetMap::Response map_res;
  while (!map_service_client.call(map_req, map_res))
  {
    ROS_ERROR("Cannot obtain the base map from the map service. Another attempt will be made.");
    ros::Duration(1.0).sleep();
    continue;
  }

  return racer_ros::msg_to_grid(map_res.map);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "racing_trajectory_planning");
  ros::NodeHandle node("~");

  std::string map_topic, state_topic, trajectory_topic, path_topic, waypoints_topic;

  node.param<std::string>("map_frame_id", map_frame_id, "map");

  node.param<std::string>("map_topic", map_topic, "/obstacles/costmap/costmap");
  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("path_visualization_topic", path_topic, "/racer/visualization/path");

  int frequency;
  node.param<int>("frequency", frequency, 1);

  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, state_update);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, waypoints_update);
  ros::Publisher trajectory_pub = node.advertise<racer_msgs::Trajectory>(trajectory_topic, 1);
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>(path_topic, 1);

  const auto actions = racer::action::create_actions(5, 9);

  ros::Rate rate(frequency);

  // blocks until map is ready
  occupancy_grid = load_map(node);

  while (ros::ok())
  {
    if (planner && last_known_state.is_valid() && !next_waypoints.empty())
    {
      std::lock_guard<std::mutex> guard(lock);
      const auto trajectory = planner->plan(
          occupancy_grid,
          last_known_state,
          actions,
          circuit,
          next_waypoint);

      if (!trajectory)
      {
        ROS_INFO("no plan found, stick to old plan");
      }
      else
      {
        nav_msgs::Path path;
        path.header = trajectory->header;

        for (const auto &step : trajectory->trajectory)
        {
          geometry_msgs::PoseStamped path_pose;
          path_pose.header = trajectory->header;
          path_pose.pose = step.pose;

          path.poses.push_back(path_pose);
        }

        ROS_INFO("publishing new plan");
        trajectory_pub.publish(*trajectory);
        path_pub.publish(path);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
