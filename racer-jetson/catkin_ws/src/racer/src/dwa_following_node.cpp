#include <stdexcept>

#include <ros/ros.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>

#include <dynamic_reconfigure/server.h>
#include <racer/DWAConfig.h>

#include "racer/action.h"
#include "racer/trajectory.h"

#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/base_model.h"
#include "racer/following_strategies/dwa.h"
#include "racer_ros/Follower.h"

using racer::vehicle_model;

std::shared_ptr<racer::following_strategies::dwa<kinematic::state>> dwa;
racer::following_strategies::trajectory_error_calculator<kinematic::state> error_calculator;

visualization_msgs::MarkerArray prepare_visualization(
    const racer_ros::Follower &follower,
    const racer::action &selected_action,
    const std::vector<racer::action> &all_actions);

void spin(
    const int frequency,
    const racer_ros::Follower &follower,
    const std::vector<racer::action> actions,
    const ros::Publisher command_pub,
    const ros::Publisher visualization_pub)
{
  ros::Rate rate(frequency);

  while (ros::ok())
  {
    if (follower.is_initialized())
    {
      auto action = follower.select_driving_command();
      if (!action.is_valid())
      {
        action = follower.stop();
      }

      ROS_DEBUG("following node selected: [throttle: %f, steering angle: %f]", action.throttle(), action.target_steering_angle());

      geometry_msgs::Twist msg;

      msg.linear.x = action.throttle();
      msg.angular.z = -action.target_steering_angle();

      command_pub.publish(msg);

      if (visualization_pub.getNumSubscribers() > 0)
      {
        const auto msg = prepare_visualization(
            follower,
            action,
            actions);
        visualization_pub.publish(msg);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void dynamic_reconfigure_callback(const racer::DWAConfig &config, uint32_t level)
{
  if (!dwa)
  {
    return;
  }

  error_calculator = {
      config.position_weight,
      config.heading_weight,
      config.velocity_weight,
      config.velocity_undershoot_overshoot_ratio,
      config.distance_to_obstacle_weight,
      config.max_position_error};

  dwa->reconfigure(error_calculator);

  ROS_INFO("DWA was reconfigured");
}

void setup_dynamic_reconfigure()
{
  dynamic_reconfigure::Server<racer::DWAConfig> server;
  dynamic_reconfigure::Server<racer::DWAConfig>::CallbackType f;
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);
  ROS_INFO("dynamic reconfiguration for DWA was set up");
}

void create_visualization_line(
    int id,
    const std::vector<racer::state> &trajectory,
    double score,
    visualization_msgs::Marker &line)
{

  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = 0.01;
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.ns = "dwa";
  line.id = id;

  line.color.r = score;
  line.color.g = 1.0 - score; // the lower, the better

  line.color.a = 0.5;

  for (const auto state : trajectory)
  {
    geometry_msgs::Point point;

    point.x = state.position().x();
    point.y = state.position().y();
    point.z = 0;

    line.points.push_back(point);
  }
}

visualization_msgs::MarkerArray prepare_visualization(
    const racer_ros::Follower &follower,
    const racer::action &selected_action,
    const std::vector<racer::action> &all_actions)
{

  visualization_msgs::MarkerArray msg;

  const auto from = follower.last_known_state();
  const auto map = follower.map();

  const auto reference_subtrajectory = follower.reference_trajectory().find_reference_subtrajectory(from, follower.next_waypoint());
  if (reference_subtrajectory.empty())
  {
    return msg;
  }

  int id = 0;
  for (const auto action : all_actions)
  {
    const auto trajectory = dwa->unfold(from, action, map);
    if (!trajectory.empty())
    {
      visualization_msgs::Marker line;
      line.header.frame_id = follower.map_frame_id;
      line.header.stamp = ros::Time::now();

      const double score = error_calculator.calculate_error(trajectory, reference_subtrajectory, map);
      create_visualization_line(id++, trajectory, score, line);

      msg.markers.push_back(line);
    }
  }

  const auto trajectory = dwa->unfold(from, selected_action, map);
  if (!trajectory.empty())
  {
    visualization_msgs::Marker selected_line;
    selected_line.header.frame_id = follower.map_frame_id;
    selected_line.header.stamp = ros::Time::now();

    const double score = error_calculator.calculate_error(trajectory, reference_subtrajectory, map);
    create_visualization_line(id++, trajectory, score, selected_line);

    selected_line.scale.x = 0.05;
    selected_line.color.a = 1.0;

    msg.markers.push_back(selected_line);
  }

  return msg;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dwa_following_node");
  ros::NodeHandle node("~");

  // setup_dynamic_reconfigure();

  double cell_size;
  std::string state_topic, trajectory_topic, waypoints_topic, map_topic, driving_topic, visualization_topic;

  node.param<double>("double", cell_size, 0.05);

  node.param<std::string>("map_topic", map_topic, "/map");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/dwa");

  double integration_step_s, prediction_horizon_s;
  node.param<double>("integration_step_s", integration_step_s, 1.0 / 20.0);
  node.param<double>("prediction_horizon_s", prediction_horizon_s, 0.5);

  auto vehicle = racer::vehicle_model::vehicle::rc_beast();
  const double radius = vehicle.radius();
  auto model = std::make_unique<racer::vehicle_model::kinematic_model>(std::move(vehicle));
  const int lookahead = static_cast<int>(ceil(prediction_horizon_s / integration_step_s));

  ROS_DEBUG("DWA strategy");
  auto actions = racer::action::create_actions_including_reverse(9, 15);

  double position_weight, heading_weight, velocity_weight, distance_to_obstacle_weight;
  node.param<double>("position_weight", position_weight, 30.0);
  node.param<double>("heading_weight", heading_weight, 20.0);
  node.param<double>("velocity_weight", velocity_weight, 10.0);
  node.param<double>("distance_to_obstacle_weight", distance_to_obstacle_weight, 5.0);

  racer::following_strategies::trajectory_error_calculator<kinematic::state> error_calculator = {
      position_weight,
      heading_weight,
      velocity_weight,
      1.0,
      distance_to_obstacle_weight,
      radius * 5};

  dwa =
      std::make_shared<racer::following_strategies::dwa<kinematic::state>>(
          lookahead,
          actions,
          std::move(model),
          error_calculator);

  ROS_DEBUG("DWA following strategy was initialized");
  racer_ros::Follower follower(dwa);

  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &racer_ros::Follower::map_observed, &follower);
  ros::Subscriber trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, &racer_ros::Follower::trajectory_observed, &follower);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, &racer_ros::Follower::waypoints_observed, &follower);
  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, &racer_ros::Follower::state_observed, &follower);

  ros::Publisher command_pub = node.advertise<geometry_msgs::Twist>(driving_topic, 1);
  ros::Publisher visualization_pub = node.advertise<visualization_msgs::MarkerArray>(visualization_topic, 1, true);

  int frequency; // Hz
  node.param<int>("update_frequency_hz", frequency, 30);

  spin(frequency, follower, actions, command_pub, visualization_pub);

  return 0;
}
