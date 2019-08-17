#include <iostream>
#include <ros/ros.h>
#include <mutex>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Waypoints.h>
#include <racer_msgs/Trajectory.h>

#include "racer/math/primitives.h"
#include "racer/vehicle_model/kinematic_bicycle_model.h"
#include "racer/sehs/space_exploration.h"

#include "racer_ros/utils.h"
#include "racer_ros/Planner.h"

std::mutex lock;

std::shared_ptr<racer::vehicle_model::kinematic_bicycle_model::state> last_known_position;
std::shared_ptr<racer::occupancy_grid> last_known_map;
std::shared_ptr<std::vector<racer::math::point>> next_waypoints;
int next_waypoint;
double waypoint_radius;
std::string map_frame;

void map_update(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  std::lock_guard<std::mutex> guard(lock);

  map_frame = map->header.frame_id;
  last_known_map = std::move(racer_ros::msg_to_grid(*map));
}

void state_update(const racer_msgs::State::ConstPtr& state) {
  std::lock_guard<std::mutex> guard(lock);

  racer::vehicle_position position(state->x, state->y, state->heading_angle);
  last_known_position = std::make_unique<racer::vehicle_model::kinematic_bicycle_model::state>(position, state->speed, state->steering_angle);
}

void waypoints_update(const racer_msgs::Waypoints::ConstPtr& waypoints) {
  std::lock_guard<std::mutex> guard(lock);

  next_waypoints = std::make_shared<std::vector<racer::math::point>>();

  waypoint_radius = waypoints->waypoints[0].radius;
  next_waypoint = waypoints->next_waypoint;

  for (const auto& wp : waypoints->waypoints) {
    next_waypoints->emplace_back(wp.position.x, wp.position.y);
  }
}

int main(int argc, char* argv[]) {
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
    0.155, // cog_offset
    0.31, // wheelbase
    0.55, // safe width
    0.75, // safe length
    2.0 / 3.0 * M_PI, // steering speed (rad/s)
    24.0 / 180.0 * M_PI, // max steering angle (rad)
    6.0, // speed (ms^-1)
    -3.0, // reversing speed (ms^-1)
    3.0 // acceleration (ms^-2)
  );

  const auto actions_with_reverse = racer::vehicle_model::kinematic_bicycle_model::action::create_actions_including_reverse(9, 5); // more throttle options, fewer steering options
  const auto actions_just_forward = racer::vehicle_model::kinematic_bicycle_model::action::create_actions(5, 9); // fewer throttle options, more steering options

  int number_of_expanded_points = 12;
  racer::astar::sehs::discretization discretization(
    vehicle.radius(), number_of_expanded_points, M_PI / 12.0, 0.25);
  
  double time_step_s = 0.1;

  racer_ros::Planner planner(
    vehicle,
    discretization,
    time_step_s,
    map_frame_id);

  ros::Rate rate(frequency);
  bool found_trajectory_last_time = true;

  while (ros::ok()) {
    if (!planner.is_initialized() && last_known_map && next_waypoints) {
      std::lock_guard<std::mutex> guard(lock);

      const std::list<racer::math::point> points{ next_waypoints->begin(), next_waypoints->end() };
      discretization.explore_grid(*last_known_map, last_known_position->position, points);
    }

    if (last_known_map && last_known_position && next_waypoints) {
      if (found_trajectory_last_time) {
        ROS_INFO("planning trajecotry just by going forward...");
      } else {
        ROS_INFO("planning trajectory with the possibility of going in reverse...");
      }

      const auto trajectory = planner.plan(
        last_known_map,
        last_known_position,
        found_trajectory_last_time ? actions_just_forward : actions_with_reverse,
        next_waypoints,
        next_waypoint,
        waypoint_radius);
      
      found_trajectory_last_time = bool(trajectory);

      if (!found_trajectory_last_time) {
        ROS_INFO("no plan found, stick to old plan");
      } else {
        nav_msgs::Path path;
        path.header = trajectory->header;

        for (const auto& step : trajectory->trajectory) {
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
