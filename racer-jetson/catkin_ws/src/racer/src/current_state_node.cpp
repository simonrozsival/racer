#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <racer_msgs/State.h>

#include "math/primitives.h"
#include "racing/vehicle_model/vehicle_position.h"

// params
std::string map_frame_id, odom_frame_id, base_link_frame_id;
double max_steering_angle;

// current state
double current_target_steering_angle;
uint32_t msg_seq;

bool is_initialized;

void command_callback(const geometry_msgs::Twist::ConstPtr& command) {
  const double target_steering_angle_percent = command->angular.z;

  current_target_steering_angle = target_steering_angle_percent * max_steering_angle;
}

double get_current_speed(const tf::TransformListener& tf_listener, const ros::Duration& averaging_interval) {
  geometry_msgs::Twist twist;

  tf_listener.lookupTwist(
    base_link_frame_id,
    odom_frame_id,
    base_link_frame_id,
    tf::Point(0, 0, 0),
    odom_frame_id,
    ros::Time(0),
    averaging_interval,
    twist);

  return twist.linear.x;
}

racing::vehicle_position get_current_position(const tf::TransformListener& tf_listener) {  
  tf::StampedTransform transform;
  tf_listener.waitForTransform(map_frame_id, base_link_frame_id, ros::Time(0), ros::Duration(0.1));
  tf_listener.lookupTransform(map_frame_id, base_link_frame_id, ros::Time(0), transform);

  auto origin = transform.getOrigin();
  auto rotation = tf::getYaw(transform.getRotation());

  return racing::vehicle_position(origin.x(), origin.y(), rotation);
}

void publish_state(
  const ros::Publisher& state_pub,
  const racing::vehicle_position& position,
  const double current_speed)
{
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
  ros::Rate rate(frequency);
  tf::TransformListener tf_listener;
  ros::Duration speed_averaging_interval(1.0 / double(frequency));
  std::string str;

  while (ros::ok()) {
    if (is_initialized) {
      try {
        const auto position = get_current_position(tf_listener);
        const auto current_speed = get_current_speed(tf_listener, speed_averaging_interval);
        publish_state(state_pub, position, current_speed);
      } catch (tf::TransformException ex) {
        ROS_ERROR("'current_state' node: TF transformation [%s -> %s] is not available (%s).", map_frame_id.c_str(), base_link_frame_id.c_str(), ex.what());
      }
    } else if (tf_listener.canTransform(map_frame_id, base_link_frame_id, ros::Time().now() - speed_averaging_interval, &str)) {
      is_initialized = true;
    } else {
      ROS_INFO("'current_state' node: TF transformation [%s -> %s] is not available yet, still waiting to be initialized (%s).", map_frame_id.c_str(), base_link_frame_id.c_str(), str.c_str());
      ros::Duration(0.5).sleep();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "current_state");
  ros::NodeHandle node("~");

  // load parameters
  std::string odom_topic, command_topic, state_topic;

  node.param<std::string>("command_topic", command_topic, "/racer/commands");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("map_frame_id", map_frame_id, "map");
  node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  node.param<std::string>("base_link_frame_id", base_link_frame_id, "base_link");

  node.param<double>("max_steering_angle", max_steering_angle, 24.0 / 180.0 * M_PI);

  int frequency;
  node.param<int>("frequency", frequency, 30);

  // reset current state
  msg_seq = 0;
  is_initialized = false;

  // set up pubsub
  ros::Subscriber command_sub = node.subscribe<geometry_msgs::Twist>(command_topic, 1, command_callback);
  ros::Publisher state_pub = node.advertise<racer_msgs::State>(state_topic, 1, false);

  spin(frequency, state_pub);

  return 0;
}
