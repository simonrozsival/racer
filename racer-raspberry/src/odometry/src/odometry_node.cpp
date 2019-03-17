#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <math>

#include "VehicleModel.h"
#include "OdometrySubject.h"

#define DRIVING_TOPIC "/racer/driving/commands"
#define WHEEL_ENCODER_TOPIC "/racer/wheel_encoders"
#define ODOMETRY_TOPIC "/racer/odometry"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle nh;

  double wheelbase = nh.getParam("wheelbase", 0.3);
  double rear_wheel_radius = nh.getParam("rear_wheel_radius", 0.1);
  double max_steering_angle = nh.getParam("max_steering_angle", 30) / 180 * M_PI;
  VehicleModel model(rear_wheel_radius, wheelbase, max_steering_angle);

  std::string base_link = nh.getParam("base_link", "base_link");
  std::string odometry_frame = nh.getParam("odometry_frame", "odom");
  ros::Publisher odometry_pub = nh.advertise<geometry_msgs::Odometry>(ODOMETRY_TOPIC, 1, true);

  OdometrySubject odometry_subject(
    model, odometry_frame, base_link, transform_broadcaster, odometry_pub);  

  ros::Subscriber steering_sub = nh.subscribe<geometry_msgs::Twist>(
    DRIVING_TOPIC, 1, &OdometrySubject::process_steering_command, &odometry_subject);

  ros::Subscriber wheel_encoders_sub = nh.subscribe<std_msgs::Float64>(
    WHEEL_ENCODER_TOPIC, 1, &OdometrySubject::process_wheel_odometry, &odometry_subject);

  ros::spin();
}
