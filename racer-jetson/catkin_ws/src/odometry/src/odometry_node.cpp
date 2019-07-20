#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <math.h>

#include "VehicleModel.h"
#include "OdometrySubject.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle nh;

  double wheelbase, rear_wheel_radius, max_steering_angle_degrees;
  
  nh.param<double>("wheelbase", wheelbase, 0.31);
  nh.param<double>("rear_wheel_radius", rear_wheel_radius, 0.05);
  nh.param<double>("max_steering_angle", max_steering_angle_degrees, 30);
  
  double max_steering_angle = max_steering_angle_degrees / 180 * M_PI;

  VehicleModel model(rear_wheel_radius, wheelbase, max_steering_angle);

  std::string base_link, odometry_frame;
  
  nh.param<std::string>("base_link", base_link, "base_link");
  nh.param<std::string>("odometry_frame", odometry_frame, "odom");

  std::string driving_topic, wheel_encoders_topic, odometry_topic;

  nh.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  nh.param<std::string>("wheel_encoders_topic", wheel_encoders_topic, "/racer/wheel_encoders");
  nh.param<std::string>("odometry_topic", odometry_topic, "/racer/odometry");

  ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>(odometry_topic, 10, true);
  tf::TransformBroadcaster odom_broadcaster;

  OdometrySubject odometry_subject(
    model, odometry_frame, base_link, odom_broadcaster, odometry_pub);  

  ros::Subscriber steering_sub = nh.subscribe<geometry_msgs::Twist>(
    driving_topic, 1, &OdometrySubject::process_steering_command, &odometry_subject);

  ros::Subscriber wheel_encoders_sub = nh.subscribe<std_msgs::Float64>(
    wheel_encoders_topic, 1, &OdometrySubject::process_wheel_odometry, &odometry_subject);

  ros::Rate loop_rate(40);

  while (ros::ok()) {
    odometry_subject.publish_odometry();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
