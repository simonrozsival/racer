#include <iostream>
#include <mutex>
#include <ros/ros.h>

#include "racer_msgs/State.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racer/action.h"
#include "racer/math.h"
#include "racer/trajectory.h"

#include "racer/astar/sehs.h"
#include "racer/sehs/space_exploration.h"
#include "racer/track/collision_detection.h"
#include "racer/track_analysis.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"

#include "racer_ros/Planner.h"
#include "racer_ros/utils.h"

using State = racer::vehicle_model::kinematic::state;
using DiscreteState = racer::astar::sehs::kinematic::discrete_state;

std::mutex mutex;

State last_known_state;
std::shared_ptr<racer::occupancy_grid> occupancy_grid;
std::shared_ptr<racer::circuit> circuit;
std::shared_ptr<racer::track::collision_detection> collision_detector;

int next_waypoint;
double waypoint_radius;
std::vector<racer::math::point> next_waypoints;

std::string map_frame_id;
std::unique_ptr<racer_ros::Planner<State, DiscreteState>> planner;

auto model = std::make_shared<racer::vehicle_model::kinematic::model>(
    racer::vehicle_model::vehicle_chassis::simulator());

double time_step_s;

void state_update(const racer_msgs::State::ConstPtr &state) {
  last_known_state = racer_ros::msg_to_state(state);
}

void waypoints_update(const racer_msgs::Waypoints::ConstPtr &waypoints) {
  std::lock_guard<std::mutex> guard(mutex);

  // have we already successfully processed these next waypoints?
  if (next_waypoint == waypoints->next_waypoint && planner) {
    return;
  }

  planner = nullptr;

  next_waypoints.clear();
  waypoint_radius = waypoints->waypoints[0].radius;
  next_waypoint = waypoints->next_waypoint;

  for (const auto &wp : waypoints->waypoints) {
    next_waypoints.emplace_back(wp.position.x, wp.position.y);
  }

  auto discretization = racer::astar::sehs::kinematic::discretization::from(
      last_known_state.configuration(), occupancy_grid, next_waypoints,
      model->chassis->radius(), model->chassis->motor->max_rpm());
  if (!discretization) {
    ROS_ERROR("space exploration failed, goal is inaccessible.");
    return;
  }

  circuit = std::make_shared<racer::circuit>(next_waypoints, waypoint_radius,
                                             occupancy_grid);
  planner = std::make_unique<racer_ros::Planner<State, DiscreteState>>(
      model, std::move(discretization), time_step_s, map_frame_id);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "racing_trajectory_planning");
  ros::NodeHandle node("~");

  std::string state_topic, trajectory_topic, path_topic, waypoints_topic;

  node.param<std::string>("map_frame_id", map_frame_id, "map");

  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("waypoints_topic", waypoints_topic,
                          "/racer/waypoints");
  node.param<std::string>("trajectory_topic", trajectory_topic,
                          "/racer/trajectory");

  int throttle_levels, steering_levels;
  node.param<int>("throttle_levels", throttle_levels, 5);
  node.param<int>("steering_levels", steering_levels, 3);

  node.param<double>("time_step_s", time_step_s, 0.1);

  ros::Subscriber state_sub =
      node.subscribe<racer_msgs::State>(state_topic, 1, state_update);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(
      waypoints_topic, 1, waypoints_update);
  ros::Publisher trajectory_pub =
      node.advertise<racer_msgs::Trajectory>(trajectory_topic, 1);

  const auto actions = racer::action::create_actions(
      throttle_levels, steering_levels, -0.4, 1.0);

  double safety_margin;
  node.param<double>("safety_margin", safety_margin, 0.3);

  // blocks until map is ready
  occupancy_grid = racer_ros::load_map(node);
  collision_detector = std::make_shared<racer::track::collision_detection>(
      occupancy_grid, model->chassis, 72, safety_margin);

  double max_frequency;
  node.param<double>("max_frequency", max_frequency, 3);
  ros::Rate rate{max_frequency};

  ROS_INFO("==> PLANNING NODE is ready to go");
  while (ros::ok()) {
    if (planner && last_known_state.is_valid() && !next_waypoints.empty()) {
      std::lock_guard<std::mutex> guard(mutex);
      const auto start_clock = std::chrono::steady_clock::now();

      const auto trajectory = planner->plan(last_known_state, actions, circuit,
                                            collision_detector, next_waypoint);

      const auto end_clock = std::chrono::steady_clock::now();
      const auto elapsed_time =
          std::chrono::duration_cast<std::chrono::milliseconds>(end_clock -
                                                                start_clock);

      if (!trajectory) {
        ROS_ERROR("X: no plan found after %ld ms, sticking to old plan for now",
                  elapsed_time.count());
      } else {
        ROS_INFO("O: trajectory planning took %ld ms", elapsed_time.count());
        trajectory_pub.publish(*trajectory);
      }
    }

    ros::spinOnce();
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
