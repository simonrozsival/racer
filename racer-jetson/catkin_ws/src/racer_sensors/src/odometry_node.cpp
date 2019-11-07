#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <math.h>

#include "racer_sensors/odometry_subject.h"
#include "racer/vehicle_model/vehicle_chassis.h"
#include "racer/vehicle_model/steering_servo_model.h"
#include "racer/vehicle_model/kinematic_model.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle nh("~");

  auto servo_model = racer::vehicle_model::steering_servo_model::with_fitted_values();
  auto vehicle = racer::vehicle_model::vehicle_chassis::rc_beast();
  auto vehicle_model = std::make_unique<racer::vehicle_model::kinematic::model>(std::move(vehicle));

  std::string base_link, odometry_frame;
  bool publish_tf;
  double gear_ratio;

  nh.param<bool>("publish_tf", publish_tf, true);
  nh.param<std::string>("base_link", base_link, "base_link");
  nh.param<std::string>("odometry_frame", odometry_frame, "odom");
  nh.param<double>("gear_ratio", gear_ratio, 3.4);

  std::string driving_topic, wheel_encoders_topic, odometry_topic;

  nh.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  nh.param<std::string>("wheel_encoders_topic", wheel_encoders_topic, "/racer/wheel_encoders");
  nh.param<std::string>("odometry_topic", odometry_topic, "/racer/odometry");

  ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry>(odometry_topic, 10, true);
  tf::TransformBroadcaster odom_broadcaster;

  odometry_subject subject(
      gear_ratio, std::move(servo_model), std::move(vehicle_model), odometry_frame, base_link, odom_broadcaster, odometry_pub);

  ros::Subscriber steering_sub = nh.subscribe<geometry_msgs::Twist>(
      driving_topic, 1, &odometry_subject::process_steering_command, &subject);

  ros::Subscriber wheel_encoders_sub = nh.subscribe<std_msgs::Float64>(
      wheel_encoders_topic, 1, &odometry_subject::process_wheel_odometry, &subject);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    subject.publish_odometry(publish_tf);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
