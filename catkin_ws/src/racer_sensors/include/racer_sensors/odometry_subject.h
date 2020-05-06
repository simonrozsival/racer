#pragma once

#include <iostream>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "racer/vehicle_model/kinematic_model.h"

class odometry_subject
{
public:
    odometry_subject(
        const double gear_ratio,
        std::unique_ptr<racer::vehicle_model::steering_servo_model> servo_model,
        std::unique_ptr<racer::vehicle_model::kinematic::model> vehicle_model,
        const std::string &odometry_frame,
        const std::string &base_link,
        tf::TransformBroadcaster &transform_broadcaster,
        ros::Publisher &odometry_topic,
        ros::Publisher &motor_rpm_topic);
    void process_steering_command(const geometry_msgs::Twist::ConstPtr &msg);
    void process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg);
    void publish_odometry(bool publish_tf);

private:
    void publish_state_estimate(
        const racer::vehicle_configuration &prediction,
        const double angular_velocity) const;

    racer::vehicle_configuration configuration_;

    double last_motor_update_time_;
    double total_revolutions_;
    double shaft_to_motor_gear_ratio_;
    double current_rpm_;

    double last_servo_update_time_;
    racer::math::angle steering_angle_;

    double last_update_time_;

    const std::unique_ptr<racer::vehicle_model::steering_servo_model> servo_model_;
    const std::unique_ptr<racer::vehicle_model::kinematic::model> vehicle_model_;

    std::mutex lock_;

    const std::string &base_link_;
    const std::string &odometry_frame_;
    const ros::Publisher &odometry_topic_;
    const ros::Publisher &motor_rpm_topic_;
    tf::TransformBroadcaster &transform_broadcaster_;
};
