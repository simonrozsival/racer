#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <racer_msgs/State.h>

#include "racing/vehicle_model/vehicle_position.h"

// params
std::string imu_topic, command_topic, map_frame_id, base_link_frame_id;
double max_steering_angle;

// current state
double last_imu_message_time_sec;
double current_speed;
double current_target_steering_angle;
uint32_t msg_seq;

bool is_initialized;

// the logic
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu) {
  const double time_sec = imu->header.stamp.sec;

  if (last_imu_message_time_sec > 0) {
    const double dt = time_sec - last_imu_message_time_sec;
    const double a = imu->linear_acceleration.x;

    current_speed += a * dt; // euler integration
  }

  last_imu_message_time_sec = time_sec;
}

void command_callback(const geometry_msgs::Twist::ConstPtr& command) {
  const double target_steering_angle_percent = command->twist.angular.z;

  current_target_steering_angle = target_steering_angle_percent * max_steering_angle;
}

racing::vehicle_position get_current_position(const tf::TransformListener& tf_listener) {  
  tf::StampedTransform transform;
  tf_listener.waitForTransform(map_frame_id, base_link_frame_id, ros::Time(0), ros::Duration(0.1));
  tf_listener.lookupTransform(map_frame_id, base_link_frame_id, ros::Time(0), transform);

  auto origin = transform.getOrigin();
  auto rotation = tf::getYaw(transform.getRotation());

  is_initialized = true;

  return racing::vehicle_position(origin.x(), origin.y(), rotation);
}

void publish_state(const ros::Publisher& state_pub, const racing::vehicle_position& position) {
    racer_msgs::State state;
    state.header.seq = msg_seq++;
    state.header.frame_id = map_frame_id;
    state.header.stamp = ros::Time::now();

    state.x = position.x;
    state.y = position.y;
    state.heading_angle = position.heading_angle;
    state.speed = current_speed;
    state.steering_angle = current_target_steering_angle;

    state_pub.publish(state);
}

void spin(const int frequency, const ros::Publisher& state_pub) {
  tf::TransformListener tf_listener;  
  ros::Rate rate(frequency);

  while (ros::ok()) {
    try {
      const auto position = get_current_position(tf_listener);
      publish_state(state_pub, position);
    } catch (tf::TransformException ex) {
      if (is_initialized) {
        ROS_ERROR("'current_state' node: TF transformation [%s -> %s] is not available.", map_frame_id, base_link_frame_id);
      } else {
        ROS_DEBUG("'current_state' node: TF transformation [%s -> %s] is not available yet.", map_frame_id, base_link_frame_id);
        ros::Duration(0.5).sleep();
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "current_state");
  ros::NodeHandle node("~");

  // load parameters
  node.param<std::string>("imu_topic", imu_topic, "/imu_data");
  node.param<std::string>("command_topic", command_topic, "/racer/commands");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("map_frame_id", map_frame_id, "map");
  node.param<std::string>("base_link_frame_id", base_link_frame_id, "base_link");

  node.param<double>("max_steering_angle", max_steering_angle, 24.0 / 180.0 * M_PI);

  int frequency;
  node.param<int>("frequency", frequency, 30);

  // reset current state
  msg_seq = 0;
  current_speed = 0;
  last_imu_message_time_sec = -1;
  is_initialized = false;

  // set up pubsub
  ros::Subscriber imu_sub = node.subscribe<sensor_msgs::Imu>(imu_topic, 1, imu_callback);
  ros::Subscriber command_sub = node.subscribe<geometry_msgs::Twist>(command_topic, 1, command_callback);
  ros::Publisher state_pub = node.advertise<racer_msgs::State>(state_topic, 1, false);

  spin(frequency, state_pub);

  return 0;
}
