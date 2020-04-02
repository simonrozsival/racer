#include <iostream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Path.h>

#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include <racer/PIDConfig.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>

#include "racer/action.h"
#include "racer/trajectory.h"

#include "racer/vehicle_model/vehicle_chassis.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/base_model.h"
#include "racer/occupancy_grid.h"
#include "racer/following_strategies/geometric_following_strategy.h"
#include "racer_ros/Follower.h"

using State = racer::vehicle_model::kinematic::state;

std::shared_ptr<racer::following_strategies::pid> pid;
std::shared_ptr<racer::vehicle_model::vehicle_chassis> vehicle;
std::shared_ptr<racer::vehicle_model::kinematic::model> model;

void pid_config_callback(racer::PIDConfig &config, uint32_t level)
{
  if (!pid)
  {
    ROS_ERROR("Cannot handle dynamic reconfiguration because the initialization of this node hasn't finished yet.");
    return;
  }

  pid->reconfigure(config.kp, config.ki, config.kd, config.error_tolerance);
  ROS_INFO("PID was reconfigured.");
}

int main(int argc, char *argv[])
{
  vehicle = racer::vehicle_model::vehicle_chassis::rc_beast();
  model = std::make_shared<racer::vehicle_model::kinematic::model>(vehicle);

  ros::init(argc, argv, "geometric_following_node");
  ros::NodeHandle node("~");

  double cell_size;
  std::string map_topic, state_topic, trajectory_topic, waypoints_topic, twist_topic, ackermann_topic, visualization_topic;

  node.param<double>("double", cell_size, 0.05);

  node.param<std::string>("map_topic", map_topic, "/map");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("twist_topic", twist_topic, "/racer/commands");
  node.param<std::string>("ackermann_topic", ackermann_topic, "/racer/ackermann_commands");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/pure_pursuit");

  ROS_DEBUG("Geometric strategy (pure pursuit + PID)");

  double max_allowed_speed_percentage;
  double kp, ki, kd, error_tolerance;
  node.param<double>("max_allowed_speed_percentage", max_allowed_speed_percentage, 1.0);
  node.param<double>("pid_speed_kp", kp, 5.0);
  node.param<double>("pid_speed_ki", ki, 0.0);
  node.param<double>("pid_speed_kd", kd, 20.0);
  node.param<double>("pid_speed_error_tolerance", error_tolerance, 0.5);
  pid = std::make_shared<racer::following_strategies::pid>(kp, ki, kd, error_tolerance);
  ROS_DEBUG("PID was initialized (kp=%f, ki=%f, kd=%f)", kp, ki, kd);

  double min_lookahead, lookahead_coefficient;
  node.param<double>("min_lookahead", min_lookahead, 1.0);
  node.param<double>("speed_lookahead_coefficient", lookahead_coefficient, 2.0);
  racer::following_strategies::pure_pursuit<State> pure_pursuit{vehicle->wheelbase, min_lookahead, lookahead_coefficient};
  ROS_DEBUG("Pure pursuit was initialized (min lookahead=%fm, lookahead velocity coefficient=%f)", min_lookahead, lookahead_coefficient);

  dynamic_reconfigure::Server<racer::PIDConfig> server;
  dynamic_reconfigure::Server<racer::PIDConfig>::CallbackType f;
  f = boost::bind(&pid_config_callback, _1, _2);
  server.setCallback(f);
  ROS_DEBUG("dynamic reconfigure for PID was set up");

  auto following_strategy = std::make_unique<racer::following_strategies::geometric_following_strategy<State>>(vehicle->steering_servo->max_steering_angle(), pid, pure_pursuit);
  ROS_DEBUG("Geometric following strategy was initialized");

  int frequency; // Hz
  node.param<int>("frequency", frequency, 30);

  racer_ros::Follower<State> follower(std::move(following_strategy), 1.0 / double(frequency));

  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &racer_ros::Follower<kinematic::state>::map_observed, &follower);  
  ros::Subscriber trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, &racer_ros::Follower<State>::trajectory_observed, &follower);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, &racer_ros::Follower<State>::waypoints_observed, &follower);
  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, &racer_ros::Follower<kinematic::state>::state_observed, &follower);

  ros::Publisher twist_pub = node.advertise<geometry_msgs::Twist>(twist_topic, 1);
  ros::Publisher ackermann_pub = node.advertise<ackermann_msgs::AckermannDrive>(ackermann_topic, 1);
  ros::Publisher visualization_pub = node.advertise<visualization_msgs::Marker>(visualization_topic, 1, true);

  ros::Rate rate(frequency);

  while (ros::ok())
  {
    if (follower.is_initialized())
    {
      auto action = follower.select_driving_command();
      std::cout << "slected action: " << action.throttle() << "x" << action.target_steering_angle() << std::endl;

      if (!action.is_valid())
      {
        action = follower.stop();
        ROS_DEBUG("following node: STOP!");
      }

      double throttle = std::min(max_allowed_speed_percentage, std::max(-max_allowed_speed_percentage, action.throttle()));
      double steering_angle = -action.target_steering_angle();

      // Twist msg
      geometry_msgs::Twist msg;
      msg.linear.x = throttle;
      msg.angular.z = steering_angle;
      twist_pub.publish(msg);

      // Ackermann msg
      ackermann_msgs::AckermannDrive ackermann_msg;
      ackermann_msg.speed = model->calculate_speed_with_no_slip_assumption(throttle * vehicle->motor->max_rpm());
      ackermann_msg.steering_angle = vehicle->steering_servo->target_steering_angle(action);
      ackermann_pub.publish(ackermann_msg);

      if (visualization_pub.getNumSubscribers() > 0)
      {
        auto pursuied_point = pure_pursuit.find_reference_position(
                                              follower.last_known_state(),
                                              follower.next_waypoint(),
                                              follower.reference_trajectory())
                                  .location();

        visualization_msgs::Marker marker;
        marker.header.frame_id = follower.map_frame_id;
        marker.header.stamp = ros::Time::now();

        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = pursuied_point.x();
        marker.pose.position.y = pursuied_point.y();
        marker.pose.position.z = 0;

        // sphere with radius 5cm
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // color depends on speed
        // @todo
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        visualization_pub.publish(marker);
      }

      ros::spinOnce();
      rate.sleep();
    }
    else
    {
      ROS_DEBUG("following node: not initialized yet");

      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }
  }

  return 0;
}
