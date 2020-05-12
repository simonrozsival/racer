#include <iostream>
#include <ros/ros.h>

#include "racer_msgs/State.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racer/math.h"
#include "racer/track/occupancy_grid.h"
#include "racer/vehicle/action.h"
#include "racer/vehicle/chassis.h"
#include "racer/vehicle/kinematic/model.h"
#include "racer/vehicle/trajectory.h"

#include "racer_ros/base_planner.h"
#include "racer_ros/hybrid_astar_planner.h"
#include "racer_ros/sehs_planner.h"
#include "racer_ros/utils.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "racing_trajectory_planning");
  ros::NodeHandle node("~");

  std::string map_topic, state_topic, trajectory_topic, path_topic,
      waypoints_topic, inflated_map_topic;

  node.param<std::string>("map_topic", map_topic, "/obstacles/costmap/costmap");
  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("waypoints_topic", waypoints_topic,
                          "/racer/waypoints");
  node.param<std::string>("trajectory_topic", trajectory_topic,
                          "/racer/trajectory");
  node.param<std::string>("inflated_map_topic", inflated_map_topic,
                          "/racer/planner_map");

  int throttle_levels, steering_levels;
  node.param<int>("throttle_levels", throttle_levels, 5);
  node.param<int>("steering_levels", steering_levels, 3);

  double time_step_s, safety_margin;
  bool use_sehs;
  node.param<double>("time_step_s", time_step_s, 0.1);
  node.param<double>("safety_margin", safety_margin, 0.1);
  node.param<bool>("use_sehs_over_hybrid_astar", use_sehs, false);

  double min_throttle, max_throttle, max_right, max_left;
  node.param<double>("min_throttle", min_throttle, -1.0);
  node.param<double>("max_throttle", max_throttle, 1.0);
  node.param<double>("max_right", max_right, -1.0);
  node.param<double>("max_left", max_left, 1.0);

  const auto actions = racer::vehicle::action::create_actions(
      throttle_levels, steering_levels, min_throttle, max_throttle, max_right,
      max_left);
  auto model = std::make_shared<racer::vehicle::kinematic::model>(
      racer::vehicle::chassis::simulator());
  auto debug_map_pub =
      node.advertise<nav_msgs::OccupancyGrid>(inflated_map_topic, 1);

  std::shared_ptr<racer_ros::base_planner> planner;

  if (use_sehs) {
    planner = std::make_shared<racer_ros::sehs_planner>(
        model, actions, time_step_s, safety_margin, debug_map_pub);
  } else {
    planner = std::make_shared<racer_ros::hybrid_astar_planner>(
        model, actions, time_step_s, safety_margin, debug_map_pub);
  }

  auto map_sub = node.subscribe<nav_msgs::OccupancyGrid>(
      map_topic, 1, [&planner](const auto &msg) { planner->map_update(msg); });
  auto state_sub = node.subscribe<racer_msgs::State>(
      state_topic, 1,
      [&planner](const auto &msg) { planner->state_update(msg); });
  auto wpts_sub = node.subscribe<racer_msgs::Waypoints>(
      waypoints_topic, 1,
      [&planner](const auto &msg) { planner->waypoints_update(msg); });

  auto trajectory_pub =
      node.advertise<racer_msgs::Trajectory>(trajectory_topic, 1);

  // blocks until map is ready
  auto map = racer_ros::load_map(node);
  planner->set_initial_map(std::move(map));

  double max_frequency, normal_frequency;
  node.param<double>("normal_frequency", normal_frequency, 1);
  node.param<double>("max_frequency", max_frequency, 10);
  ros::Rate normal_rate{normal_frequency};
  ros::Rate max_rate{max_frequency};

  ROS_INFO("==> PLANNING NODE is ready to go");

  while (ros::ok()) {
    if (planner->is_ready()) {
      const auto start_clock = std::chrono::steady_clock::now();

      const auto trajectory = planner->plan();

      const auto end_clock = std::chrono::steady_clock::now();
      const auto elapsed_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_clock -
                                                                start_clock);

      if (!trajectory) {
        ROS_INFO("X: no plan found after %ld ms, sticking to old plan for now",
                 elapsed_time.count());
      } else {
        ROS_INFO("O: trajectory planning took %ld ms", elapsed_time.count());
        trajectory_pub.publish(*trajectory);
        normal_rate.sleep();
      }
    }

    max_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
