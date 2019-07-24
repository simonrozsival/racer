#include <iostream>
#include <ros/ros.h>
#include <mutex>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "racer_msgs/Waypoints.h"
#include "racer_msgs/Trajectory.h"

#include "math/primitives.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"
#include "Planner.h"

std::mutex lock;

nav_msgs::Odometry last_known_position;
nav_msgs::OccupancyGrid last_known_map;
racer_msgs::Waypoints next_waypoints;

bool has_map = false;
bool has_odom = false;
bool has_goal = false;

void map_update(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  std::lock_guard<std::mutex> guard(lock);
  last_known_map = *map;
  has_map = true;
}

void odometry_update(const nav_msgs::Odometry::ConstPtr& position) {
  std::lock_guard<std::mutex> guard(lock);
  last_known_position = *position;
  has_odom = true;
}

void waypoints_update(const racer_msgs::Waypoints::ConstPtr& waypoints) {
  std::lock_guard<std::mutex> guard(lock);
  next_waypoints = *waypoints;
  has_goal = true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "racing_trajectory_planning");
  ros::NodeHandle node;

  std::string map_topic, odometry_topic, trajectory_topic, path_topic, waypoints_topic;

  node.param<std::string>("map_topic", map_topic, "/map");
  node.param<std::string>("odometry_topic", odometry_topic, "/pf/pose/odom");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("path_visualization_topic", path_topic, "/racer/visualization/path");

  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, map_update);
  ros::Subscriber odometry_sub = node.subscribe<nav_msgs::Odometry>(odometry_topic, 1, odometry_update);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, waypoints_update);
  ros::Publisher trajectory_pub = node.advertise<racer_msgs::Trajectory>(trajectory_topic, 1);
  ros::Publisher path_pub = node.advertise<nav_msgs::Path>(path_topic, 1);
  
  racing::vehicle vehicle(
    0.155, // cog_offset
    0.31, // wheelbase
    0.35, // safe width
    0.55, // safe length
    2.0 / 3.0 * M_PI, // steering speed (rad/s)
    1.0 / 6.0 * M_PI, // max steering angle (rad)
    2.0, // speed (ms^-1)
    2.0 // acceleration (ms^-2)
  );

  auto actions = racing::kinematic_model::action::create_actions(3, 9);
  astar::hybrid_astar::discretization discretization(
    1.0, 1.0, M_PI / 12.0, 0.25);
  
  Planner planner(
    vehicle,
    actions,
    discretization);

  ros::Rate rate(1);

  while (ros::ok()) {
    if (!planner.is_initialized() && has_map) {
      planner.initialize(last_known_map);
    }

    if (has_map && has_odom && has_goal) {
      ROS_INFO("planning...");

      nav_msgs::Odometry odom;
      racer_msgs::Waypoints waypoints;
      nav_msgs::OccupancyGrid map;
      {
        std::lock_guard<std::mutex> guard(lock);
        odom = last_known_position;
        waypoints = next_waypoints;
        map = last_known_map;
      }

      const auto trajectory = planner.plan(map, odom, waypoints);
      
      if (!trajectory) {
        ROS_INFO("no plan found, sticking to old plan");
        continue;
      }

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

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
