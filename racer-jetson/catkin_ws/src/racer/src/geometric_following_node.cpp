#include "ros/ros.h"
#include <cmath>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include "visualization_msgs/Marker.h"

#include <dynamic_reconfigure/server.h>
#include <racer/PIDConfig.h>

#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racing/vehicle_model/kinematic_bicycle_model.h"
#include "racing/vehicle_model/base_vehicle_model.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"
#include "racing/following_strategies/geometric_following_strategy.h"
#include "math/euler_method_integrator.h"
#include "Follower.h"

std::shared_ptr<racing::pid> pid;

void pid_config_callback(racer::PIDConfig &config, uint32_t level) {
  if (!pid) return;
  pid->reconfigure(config.kp, config.ki, config.kd, config.error_tolerance);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "geometric_following_node");
  ros::NodeHandle node;

  double cell_size;
  std::string odometry_topic, trajectory_topic, waypoints_topic, costmap_topic, driving_topic, visualization_topic;

  node.param<double>("double", cell_size, 0.05);

  node.param<std::string>("costmap_topic", costmap_topic, "/costmap");
  node.param<std::string>("odometry_topic", odometry_topic, "/pf/pose/odom");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");

  node.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/pure_pursuit");

  const double wheelbase = 0.31; // m

  std::cout << "Geometric strategy (pure pursuit + PID)" << std::endl;
  double kp, ki, kd, error_tolerance;
  node.param<double>("pid_speed_kp", kp, 1.0);
  node.param<double>("pid_speed_ki", ki, 0.0);
  node.param<double>("pid_speed_kd", kd, 1.0);
  node.param<double>("pid_speed_error_tolerance", error_tolerance, 0.5);
  pid = std::make_shared<racing::pid>(kp, ki, kd, error_tolerance);
  std::cout << "PID was initialized (kp=" << kp << ", ki=" << ki << ", kd=" << kd << ")" << std::endl;

  double min_lookahead, lookahead_coef;
  node.param<double>("min_lookahead", min_lookahead, wheelbase * 5.0);
  node.param<double>("speed_lookahead_coef", lookahead_coef, 2.0);
  auto pure_pursuit = std::make_shared<racing::pure_pursuit>(wheelbase, min_lookahead, lookahead_coef);
  std::cout << "Pure pursuit was initialized (min lookahead=" << min_lookahead << "m, lookahead velocity coef=" << lookahead_coef << ")" << std::endl;

  dynamic_reconfigure::Server<racer::PIDConfig> server;
  dynamic_reconfigure::Server<racer::PIDConfig>::CallbackType f;  
  f = boost::bind(&pid_config_callback, _1, _2);
  server.setCallback(f);
  std::cout << "dynamic reconfigure for PID was set up" << std::endl;

  auto following_strategy = std::make_unique<racing::geometric_following_strategy>(pid, pure_pursuit);
  std::cout << "Geometric following strategy was initialized" << std::endl;

  Follower follower(std::move(following_strategy));
  
  ros::Subscriber costmap_sub = node.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 1, &Follower::costmap_observed, &follower);
  ros::Subscriber odometry_sub = node.subscribe<nav_msgs::Odometry>(odometry_topic, 1, &Follower::state_observed, &follower);
  ros::Subscriber trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, &Follower::trajectory_observed, &follower);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, &Follower::waypoints_observed, &follower);

  ros::Publisher command_pub = node.advertise<geometry_msgs::Twist>(driving_topic, 1);
  ros::Publisher visualization_pub = node.advertise<visualization_msgs::Marker>(visualization_topic, 1, true);

  int frequency; // Hz
  node.param<int>("update_frequency_hz", frequency, 5);

  ros::Rate rate(frequency);

  while (ros::ok()) {
    if (follower.is_initialized()) {
      auto action = follower.select_driving_command();

      if (!action) {
        action = follower.stop();
        std::cout << "following node: STOP!" << std::endl;
      } else {
        std::cout << "following node selected: [throttle: " << action->throttle << ", steering angle: " << action->target_steering_angle << "]" << std::endl;
      }

      geometry_msgs::Twist msg;
      msg.linear.x = action->throttle;
      msg.angular.z = action->target_steering_angle;

      command_pub.publish(msg);

      if (visualization_pub.getNumSubscribers() > 0) {
        auto pursuied_point = pure_pursuit->find_reference_position(
          follower.last_known_state(),
          follower.next_waypoint(),
          follower.reference_trajectory()
        ).location();

        visualization_msgs::Marker marker;
        marker.header.frame_id = follower.frame_id;
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
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
