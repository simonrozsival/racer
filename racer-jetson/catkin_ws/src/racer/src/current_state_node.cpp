#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <racer_msgs/State.h>

#include "racer/math/primitives.h"
#include "racer/vehicle_position.h"

// params
std::string map_frame_id, odom_frame_id, base_link_frame_id;
double max_steering_angle;

// current state
std::unique_ptr<racer::vehicle_position> prev_position;
double prev_position_time;
ros::Time prev_transform_stamp;

double current_target_steering_angle;
uint32_t msg_seq;

bool is_initialized;

void command_callback(const geometry_msgs::Twist::ConstPtr& command) {
  const double target_steering_angle_percent = command->angular.z;

  current_target_steering_angle = target_steering_angle_percent * max_steering_angle;
}

double fix_angle(double angle) {
  while (angle < 0) angle += 2 * M_PI;
  while (angle > 2 * M_PI) angle -= 2 * M_PI;
  return angle;
}

double angle_difference(double alpha, double beta) {
  alpha = fix_angle(alpha);
  beta = fix_angle(beta);

  return std::min(fix_angle(alpha - beta), fix_angle(beta - alpha));
}

double get_current_speed(const racer::vehicle_position current_position) {
  double now = ros::Time().now().toSec();
  double current_speed = 0;

  if (prev_position) {
    racer::math::vector delta = current_position.location() - prev_position->location();
    double distance = delta.length();
    double travel_angle = atan2(delta.y, delta.x);
    double direction = angle_difference(travel_angle, prev_position->heading_angle) < (M_PI / 2) ? 1.0 : -1.0;
    double elapsed_time = now - prev_position_time;

    current_speed = direction * distance / elapsed_time;

    if (std::abs(current_speed) < 0.001) {
      current_speed = 0;
    }
  }

  prev_position_time = now;
  prev_position = std::make_unique<racer::vehicle_position>(current_position);

  return current_speed;
}

racer::vehicle_position get_current_position(const tf::Transform& transform) {
  auto origin = transform.getOrigin();
  auto rotation = tf::getYaw(transform.getRotation());

  return racer::vehicle_position(origin.x(), origin.y(), rotation);
}

void publish_state(
  const ros::Publisher& state_pub,
  const racer::vehicle_position& position,
  const double current_speed)
{
  racer_msgs::State state;
  state.header.seq = msg_seq++;
  state.header.frame_id = odom_frame_id;
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
  std::string err;

  while (ros::ok()) {
    if (is_initialized) {
      try {
        tf::StampedTransform odom_to_base_link, map_to_odom;
        tf_listener.lookupTransform(odom_frame_id, base_link_frame_id, ros::Time(0), odom_to_base_link);
        tf_listener.lookupTransform(map_frame_id, odom_frame_id, ros::Time(0), map_to_odom);

        if (odom_to_base_link.stamp_ != prev_transform_stamp) {
          const auto position = get_current_position(map_to_odom * odom_to_base_link);
          const auto current_speed = get_current_speed(position);
          publish_state(state_pub, position, current_speed);
          prev_transform_stamp = odom_to_base_link.stamp_;
        }

      } catch (tf::TransformException ex) {
        ROS_ERROR("'current_state' node: %s.", ex.what());
      }
    } else if (tf_listener.canTransform(map_frame_id, base_link_frame_id, ros::Time(0), &err)) {
      is_initialized = true;
      ROS_INFO("'current_state' node: INITIALIZED.");
    } else {
      ROS_INFO("'current_state' node: Still not initialized (%s).", err.c_str());
      err = "";
      ros::Duration(1.0).sleep();
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
