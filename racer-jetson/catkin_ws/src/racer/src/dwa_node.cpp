#include <stdexcept>

#include <ros/ros.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <tf/transform_datatypes.h>

#include <ackermann_msgs/AckermannDrive.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>

#include "racer/action.h"
#include "racer/trajectory.h"

#include "racer/following_strategies/dwa.h"
#include "racer/vehicle_model/base_model.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"
#include "racer_ros/Follower.h"

using kinematic_model = racer::vehicle_model::kinematic::model;
using kinematic_state = racer::vehicle_model::kinematic::state;
using Follower = racer_ros::Follower<kinematic_state>;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "dwa_node");
  ros::NodeHandle node("~");

  std::string state_topic, trajectory_topic, waypoints_topic, twist_topic, ackermann_topic;
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("twist_topic", twist_topic, "/racer/commands");
  node.param<std::string>("ackermann_topic", ackermann_topic, "/racer/ackermann_commands");

  double integration_step_s, prediction_horizon_s;
  node.param<double>("integration_step_s", integration_step_s, 1.0 / 25.0);
  node.param<double>("prediction_horizon_s", prediction_horizon_s, 0.5);

  const auto model = std::make_shared<kinematic_model>(racer::vehicle_model::vehicle_chassis::rc_beast());
  const int lookahead = static_cast<int>(ceil(prediction_horizon_s / integration_step_s));

  auto actions = racer::action::create_actions(5, 15);
  double position_weight, heading_weight, velocity_weight, distance_to_obstacle_weight;
  node.param<double>("position_weight", position_weight, 30.0);
  node.param<double>("heading_weight", heading_weight, 20.0);
  node.param<double>("velocity_weight", velocity_weight, 10.0);
  node.param<double>("distance_to_obstacle_weight", distance_to_obstacle_weight, 5.0);

  const racer::following_strategies::trajectory_error_calculator<kinematic_state> error_calculator = {
    position_weight, heading_weight, velocity_weight, distance_to_obstacle_weight, model->chassis->motor->max_rpm()
  };
  const racer::following_strategies::unfolder<racer::vehicle_model::kinematic::state> unfolder{ model,
                                                                                                integration_step_s,
                                                                                                lookahead };
  const auto dwa =
      std::make_shared<racer::following_strategies::dwa<kinematic_state>>(actions, unfolder, error_calculator);
  Follower follower{ racer_ros::load_map(node), dwa, integration_step_s };

  auto trajectory_sub =
      node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, &Follower::trajectory_observed, &follower);
  auto waypoints_sub =
      node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, &Follower::waypoints_observed, &follower);
  auto state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, &Follower::state_observed, &follower);

  auto twist_pub = node.advertise<geometry_msgs::Twist>(twist_topic, 1);
  auto ackermann_pub = node.advertise<ackermann_msgs::AckermannDrive>(ackermann_topic, 1);

  double frequency;  // Hz
  node.param<double>("update_frequency_hz", frequency, 25);
  ros::Rate rate(frequency);

  ROS_INFO("==> DWA NODE is ready to go");
  while (ros::ok())
  {
    if (follower.is_initialized())
    {
      auto action = follower.select_driving_command();
      if (!action.is_valid())
      {
        action = follower.stop();
      }

      twist_pub.publish(racer_ros::action_to_twist_msg(action));
      ackermann_pub.publish(racer_ros::action_to_ackermann_msg(action, model));
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
