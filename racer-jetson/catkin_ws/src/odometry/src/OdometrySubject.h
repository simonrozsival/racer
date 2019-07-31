#ifndef ODOMETRY_SUBJECT_H_
#define ODOMETRY_SUBJECT_H_

#include <iostream>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include "VehicleState.h"
#include "VehicleModel.h"

class OdometrySubject {
    public:
        OdometrySubject(
            const VehicleModel& vehicle_model,
            const std::string& odometry_frame,
            const std::string& base_link,
            tf::TransformBroadcaster& transform_broadcaster,
            ros::Publisher& odometry_topic);
        void process_steering_command(const geometry_msgs::Twist::ConstPtr &msg);
        void process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg);
        void publish_odometry();

    private:
        void publish_state_estimate(const VehicleState &state) const;
  
        VehicleState state_;
        double total_distance_last_time_;
        double total_distance_;
        double steering_angle_;
        double last_update_time_;
        double direction_;

        std::mutex lock_;

        const VehicleModel& vehicle_model_;
        const std::string& base_link_;
        const std::string& odometry_frame_;
        const ros::Publisher& odometry_topic_;
        tf::TransformBroadcaster& transform_broadcaster_;
};

#endif
