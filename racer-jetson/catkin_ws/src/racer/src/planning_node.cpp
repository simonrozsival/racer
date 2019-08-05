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
#include "sehs/space_exploration.h"

#include "utils.h"
#include "Planner.h"

std::mutex lock;

std::shared_ptr<racing::kinematic_model::state> last_known_position;
std::shared_ptr<racing::occupancy_grid> last_known_map;
std::shared_ptr<std::vector<math::point>> next_waypoints;
int next_waypoint;
double waypoint_radius;
std::string map_frame;

void map_update(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  std::lock_guard<std::mutex> guard(lock);

  map_frame = map->header.frame_id;
  last_known_map = std::move(msg_to_grid(*map));
}

void odometry_update(const nav_msgs::Odometry::ConstPtr& odom) {
  std::lock_guard<std::mutex> guard(lock);

  last_known_position = std::move(msg_to_state(*odom));
}

void waypoints_update(const racer_msgs::Waypoints::ConstPtr& waypoints) {
  std::lock_guard<std::mutex> guard(lock);

  next_waypoints = std::make_shared<std::vector<math::point>>();

  waypoint_radius = waypoints->waypoints[0].radius;
  next_waypoint = waypoints->next_waypoint;

  for (const auto& wp : waypoints->waypoints) {
    next_waypoints->emplace_back(wp.position.x, wp.position.y);
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "racing_trajectory_planning");
  ros::NodeHandle node("~");

  std::string map_topic, odometry_topic, trajectory_topic, path_topic, waypoints_topic;

  node.param<std::string>("map_topic", map_topic, "/map");
  node.param<std::string>("odometry_topic", odometry_topic, "/pf/pose/odom");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("path_visualization_topic", path_topic, "/racer/visualization/path");

  bool allow_reverse;
  node.param<bool>("allow_reverse", allow_reverse, false);

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
    24.0 / 180.0 * M_PI, // max steering angle (rad)
    2.0, // speed (ms^-1)
    2.0 // acceleration (ms^-2)
  );

  auto actions = allow_reverse
    ? racing::kinematic_model::action::create_actions(3, 9)
    : racing::kinematic_model::action::create_actions_including_reverse(5, 9);

  int number_of_expanded_points = 12;
  astar::sehs::discretization discretization(
    vehicle.radius(), number_of_expanded_points, M_PI / 12.0, 0.25);
  
  Planner planner(
    vehicle,
    actions,
    discretization);

  ros::Rate rate(10);

  while (ros::ok()) {
    if (!planner.is_initialized() && last_known_map) {
      std::lock_guard<std::mutex> guard(lock);

      const std::list<math::point> points{ next_waypoints->begin(), next_waypoints->end() };
      discretization.explore_grid(*last_known_map, last_known_position->position, points);
      planner.initialize(last_known_map->cell_size, map_frame);
    }

    if (last_known_map && last_known_position && next_waypoints) {
      ROS_INFO("planning...");

      const auto trajectory = planner.plan(
        last_known_map,
        last_known_position,
        next_waypoints,
        next_waypoint,
        waypoint_radius);
      
      if (!trajectory) {
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
