#include "ros/ros.h"
#include <cmath>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include <racer/PIDConfig.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>

#include "racing/vehicle_model/kinematic_bicycle_model.h"
#include "racing/vehicle_model/base_vehicle_model.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"
#include "racing/following_strategies/geometric_following_strategy.h"
#include "math/euler_method_integrator.h"
#include "Follower.h"

std::shared_ptr<racing::pid> pid;

void pid_config_callback(racer::PIDConfig &config, uint32_t level) {
  if (!pid) {
    ROS_ERROR("Cannot handle dynamic reconfiguration because the initialization of this node hasn't finished yet.");
    return;
  }

  pid->reconfigure(config.kp, config.ki, config.kd, config.error_tolerance);
  ROS_INFO("PID was reconfigured.");
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "geometric_following_node");
  ros::NodeHandle node("~");

  double cell_size;
  std::string state_topic, trajectory_topic, waypoints_topic, driving_topic, visualization_topic;

  node.param<double>("double", cell_size, 0.05);

  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/pure_pursuit");

  const double wheelbase = 0.31; // m
  const double max_steering_angle = 24.0 / 180.0 * M_PI;

  ROS_DEBUG("Geometric strategy (pure pursuit + PID)");

  double max_allowed_speed_percentage;
  double kp, ki, kd, error_tolerance;
  node.param<double>("max_allowed_speed_percentage", max_allowed_speed_percentage, 1.0);
  node.param<double>("pid_speed_kp", kp, 1.0);
  node.param<double>("pid_speed_ki", ki, 0.0);
  node.param<double>("pid_speed_kd", kd, 1.0);
  node.param<double>("pid_speed_error_tolerance", error_tolerance, 0.5);
  pid = std::make_shared<racing::pid>(kp, ki, kd, error_tolerance);
  ROS_DEBUG("PID was initialized (kp=%f, ki=%f, kd=%f)", kp, ki, kd);

  double min_lookahead, lookahead_coef;
  node.param<double>("min_lookahead", min_lookahead, 1.0);
  node.param<double>("speed_lookahead_coef", lookahead_coef, 2.0);
  auto pure_pursuit = std::make_shared<racing::pure_pursuit>(wheelbase, min_lookahead, lookahead_coef);
  ROS_DEBUG("Pure pursuit was initialized (min lookahead=%fm, lookahead velocity coef=%f)", min_lookahead, lookahead_coef);

  dynamic_reconfigure::Server<racer::PIDConfig> server;
  dynamic_reconfigure::Server<racer::PIDConfig>::CallbackType f;  
  f = boost::bind(&pid_config_callback, _1, _2);
  server.setCallback(f);
  ROS_DEBUG("dynamic reconfigure for PID was set up");

  auto following_strategy = std::make_unique<racing::geometric_following_strategy>(max_steering_angle, pid, pure_pursuit);
  ROS_DEBUG("Geometric following strategy was initialized");

  Follower follower(std::move(following_strategy));
  
  ros::Subscriber trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, &Follower::trajectory_observed, &follower);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, &Follower::waypoints_observed, &follower);

  ros::Publisher command_pub = node.advertise<geometry_msgs::Twist>(driving_topic, 1);
  ros::Publisher visualization_pub = node.advertise<visualization_msgs::Marker>(visualization_topic, 1, true);

  int frequency; // Hz
  node.param<int>("frequency", frequency, 30);

  ros::Rate rate(frequency);

  while (ros::ok()) {
    if (follower.is_initialized()) {
      auto action = follower.select_driving_command();

      if (!action) {
        action = follower.stop();
        ROS_DEBUG("following node: STOP!");
      }

      double throttle = std::min(max_allowed_speed_percentage, std::max(-max_allowed_speed_percentage, action->throttle));
      double steering_angle = -action->target_steering_angle;

      geometry_msgs::Twist msg;
      msg.linear.x = throttle;
      msg.angular.z = steering_angle;

      command_pub.publish(msg);

      if (visualization_pub.getNumSubscribers() > 0) {
        auto pursuied_point = pure_pursuit->find_reference_position(
          follower.last_known_state(),
          follower.next_waypoint(),
          follower.reference_trajectory()
        ).location();

        visualization_msgs::Marker marker;
        marker.header.frame_id = follower.map_frame_id;
        marker.header.stamp = ros::Time::now();

        marker.ns = "pure_pursuit";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
    
        marker.pose.position.x = pursuied_point.x;
        marker.pose.position.y = pursuied_point.y;
        marker.pose.position.z = 0;

        // sphere with radius 5cm
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        visualization_pub.publish(marker);
      }
  
      ros::spinOnce();      
      rate.sleep();
    } else {
      ROS_DEBUG("following node: not initialized yet");

      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }
  }

  return 0;
}
