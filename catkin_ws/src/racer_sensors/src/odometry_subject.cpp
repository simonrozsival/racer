#include <ros/ros.h>

#include <math.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "racer_sensors/odometry_subject.h"

odometry_subject::odometry_subject(
    const double gear_ratio,
    std::unique_ptr<racer::vehicle_model::steering_servo_model> servo_model,
    std::unique_ptr<racer::vehicle_model::kinematic::model> vehicle_model,
    const std::string &odometry_frame,
    const std::string &base_link,
    tf::TransformBroadcaster &transform_broadcaster,
    ros::Publisher &odometry_topic,
    ros::Publisher &motor_rpm_topic)
    : configuration_{},
      last_motor_update_time_{0},
      total_revolutions_{0},
      shaft_to_motor_gear_ratio_{gear_ratio},
      last_servo_update_time_{0},
      steering_angle_{0},
      last_update_time_{0},
      servo_model_{std::move(servo_model)},
      vehicle_model_{std::move(vehicle_model)},
      base_link_{base_link},
      odometry_frame_{odometry_frame},
      transform_broadcaster_{transform_broadcaster},
      odometry_topic_{odometry_topic},
      motor_rpm_topic_{motor_rpm_topic}
{
    last_servo_update_time_ = ros::Time::now().toSec();
    last_motor_update_time_ = ros::Time::now().toSec();
}

void odometry_subject::process_steering_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    const double t = ros::Time::now().toSec();
    const double dt = t - last_servo_update_time_;

    const auto action = racer::action{0, msg->angular.z}; // we don't care about the throttle
    steering_angle_ = servo_model_->predict_next_state(steering_angle_, action, dt);

    last_servo_update_time_ = t;
}

void odometry_subject::process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg)
{
    const double t = ros::Time::now().toSec();
    const double dt = t - last_motor_update_time_;
    const double revs = shaft_to_motor_gear_ratio_ * msg->data;

    const auto delta_revolutions = revs - total_revolutions_;
    const auto revolutions_per_second = delta_revolutions / dt;
    current_rpm_ = revolutions_per_second * 60;

    total_revolutions_ = revs;
    last_motor_update_time_ = t;

    std_msgs::Float64 rpm_msg;
    rpm_msg.data = current_rpm_;
    motor_rpm_topic_.publish(rpm_msg);
}

void odometry_subject::publish_odometry(bool publish_tf)
{
    std::lock_guard<std::mutex> guard(lock_);

    const double current_time = ros::Time::now().toSec();
    const double elapsed_time = current_time - last_update_time_;
    const auto current_state = racer::vehicle_model::kinematic::state{configuration_, current_rpm_, steering_angle_};

    const auto prediction = vehicle_model_->predict_next_state(current_state, racer::action{0, 0}, elapsed_time);
    const auto angular_velocity = configuration_.heading_angle().distance_to(prediction.configuration().heading_angle()) / elapsed_time;

    publish_state_estimate(prediction.configuration(), angular_velocity);
    configuration_ = prediction.configuration();

    last_update_time_ = ros::Time::now().toSec();

    if (publish_tf)
    {
        //since all odometry is 6DOF we'll need a quaternion created from yaw$
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(configuration_.heading_angle());

        //first, we'll publish the transform over tf$
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = odometry_frame_;
        odom_trans.child_frame_id = base_link_;

        odom_trans.transform.translation.x = configuration_.location().x();
        odom_trans.transform.translation.y = configuration_.location().y();
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform$
        transform_broadcaster_.sendTransform(odom_trans);
    }
}

void odometry_subject::publish_state_estimate(
    const racer::vehicle_configuration &predicted_configuration,
    const double angular_velocity) const
{
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odometry_frame_;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(predicted_configuration.heading_angle());

    //set the position
    odom.pose.pose.position.x = predicted_configuration.location().x();
    odom.pose.pose.position.y = predicted_configuration.location().y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = base_link_;
    odom.twist.twist.linear.x = vehicle_model_->calculate_speed_with_no_slip_assumption(current_rpm_);
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = angular_velocity;

    //set covariance
    odom.pose.covariance[0] = 0.2;   // x
    odom.pose.covariance[7] = 0.2;   // y
    odom.pose.covariance[14] = 9999; // z
    odom.pose.covariance[21] = 9999; // roll
    odom.pose.covariance[28] = 9999; // pitch
    odom.pose.covariance[35] = 0.4;  // yaw

    //publish the message
    odometry_topic_.publish(odom);
}
