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
#include "racer/sehs/space_exploration.h"

#include "racer_ros/utils.h"
#include "racer_ros/Planner.h"

std::mutex lock;

racer::vehicle_model::kinematic::state last_known_position;
std::vector<racer::math::point> next_waypoints;
int next_waypoint;
double waypoint_radius;
std::string map_frame;
std::unique_ptr<racer_ros::Planner> planner;

void state_update(const racer_msgs::State::ConstPtr &state)
{
  std::lock_guard<std::mutex> guard(lock);

  racer::vehicle_configuration position{state->x, state->y, state->heading_angle};
  last_known_position = {position, state->speed, state->steering_angle};
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

  planner = nullptr;
}

racer::occupancy_grid load_map()
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

  std::string map_frame_id, map_topic, state_topic, trajectory_topic, path_topic, waypoints_topic;

  node.param<std::string>("map_frame_id", map_frame_id, "map");

  node.param<std::string>("map_topic", map_topic, "/obstacles/costmap/costmap");
  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("path_visualization_topic", path_topic, "/racer/visualization/path");

  int frequency;
  node.param<int>("frequency", frequency, 1);

  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, map_update);
  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, state_update);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, waypoints_update);
  ros::Publisher trajectory_pub = node.advertise<racer_msgs::Trajectory>(trajectory_topic, 1);
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>(path_topic, 1);

  racer::vehicle_model::vehicle vehicle(
      0.155,               // cog_offset
      0.31,                // wheelbase
      0.55,                // safe width
      0.75,                // safe length
      2.0 / 3.0 * M_PI,    // steering speed (rad/s)
      24.0 / 180.0 * M_PI, // max steering angle (rad)
      6.0,                 // speed (ms^-1)
      -3.0,                // reversing speed (ms^-1)
      3.0                  // acceleration (ms^-2)
  );

  const auto actions_with_reverse = racer::action::create_actions_including_reverse(9, 5); // more throttle options, fewer steering options
  const auto actions_just_forward = racer::action::create_actions(5, 9);                   // fewer throttle options, more steering options

  int number_of_expanded_points = 12;
  double time_step_s = 1.0 / 25.0;

  ros::Rate rate(frequency);
  bool found_trajectory_last_time = true;

  // blocks until map is ready
  const auto map = load_map();

  while (ros::ok())
  {
    if (!planner && !next_waypoints.empty() && last_known_position.is_valid())
    {
      std::lock_guard<std::mutex> guard(lock);

      racer::sehs::space_exploration exploration(config->occupancy_grid, vehicle.radius(), 2 * vehicle.radius(), config->neighbor_circles);
      const auto path_of_circles = exploration.explore_grid(last_known_position, next_waypoints);
      auto discretization = std::make_unique<racer::astar::sehs::discretization>(path_of_circles, M_PI / 12.0, 0.25);
      planner = std::make_unique<racer_ros::Planner>(
          vehicle,
          std::move(discretization),
          time_step_s,
          map_frame_id)
    }

    if (planner && last_known_position.is_valid() && !next_waypoints.empty())
    {
      if (found_trajectory_last_time)
      {
        ROS_INFO("planning trajecotry just by going forward...");
      }
      else
      {
        ROS_INFO("planning trajectory with the possibility of going in reverse...");
      }

      std::lock_guard<std::mutex> guard(lock);
      const auto trajectory = planner.plan(
          last_known_map,
          last_known_position,
          found_trajectory_last_time ? actions_just_forward : actions_with_reverse,
          next_waypoints,
          next_waypoint,
          waypoint_radius);

      found_trajectory_last_time = bool(trajectory);

      if (!found_trajectory_last_time)
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
