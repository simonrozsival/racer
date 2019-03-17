#ifndef ODOMETRY_SUBJECT_H_
#define ODOMETRY_SUBJECT_H_

class OdometrySubject {
    public:
        OdometrySubject(
            std::string odometry_frame,
            std::string base_link,
            tf::TransformBroadcaster transform_broadcaster,
            ros::Publisher odometry_topic);
        void process_steering_command(const geometry_msgs::Twist::ConstPtr &msg);
        void process_wheel_odometry(const std_msgs::Float64::ConstPtr &msg);

    private:
        void publish_state_estimate(const VehicleState &state);
  
        VehicleState state_;
        double last_update_time_;

        std::string base_link_;
        std::string odometry_frame_;
        ros::Publisher odometry_topic_;
};

#endif