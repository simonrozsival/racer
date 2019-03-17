#include "OdometrySubject.h"
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

OdometrySubject::OdometrySubject(
    VehicleModel vehicle_model,
    std::string odometry_frame,
    std::string base_link,
    tf::TransformBroadcaster transform_broadcaster,
    ros::Publisher odometry_topic)
    : vehicle_model_(vehicle_model),
      base_link_(base_link),
      odometry_frame_(odometry_frame),
      transform_broadcaster_(transform_broadcaster),
      odometry_topic_(odometry_topic),
      total_distance_(0),
      steering_angle_(0)
{
    last_update_time_ = ros::Time::now();

    state_ = VehicleState();
    state_.x = 0;
    state_.y = 0;
    state_.heading_angle = 0;
    state_.vx = 0;
    state_.vy = 0;
    state_.angular_velocity = 0;
}

void OdometrySubject::process_steering_command(const geometry_msgs::Twist::ConstPtr &msg)
{
    steering_angle_ = vehicle_model.max_steering_angle() * msg.angular.z;    
}

void OdometrySubject::process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg)
{
    double revolutions = msg.data;
    double distance = revolutions * rear_wheel_radius_ * M_PI;
    double current_time = ros::Time::now();
    
    double step = distance - total_distance_;
    double elapsed_time = current_time - last_update_time_;

    vehicle_model_.update_state(state_, step, steering_angle_, elapsed_time);
    publish_state_estimate(last_estimated_state_);

    total_distance_ = distance;
    last_update_time_ = current_time;
}

void OdometrySubject::publish_state_estimate(const VehicleState &state)
{  
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odometry_frame_;

    //set the position
    odom.pose.pose.position.x = state.x;
    odom.pose.pose.position.y = state.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = base_link_;
    odom.twist.twist.linear.x = state.vx;
    odom.twist.twist.linear.y = state.vy;
    odom.twist.twist.angular.z = state.angular_velocity;

    //publish the message
    odom_pub.publish(odom);
}
