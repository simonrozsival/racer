#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <racer_msgs/State.h>
#include <std_msgs/Float64.h>

#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"

#include "racer_ros/utils.h"

// params
std::string map_frame_id, odom_frame_id, base_link_frame_id;

auto chassis = racer::vehicle_model::vehicle_chassis::simulator();

// motor
double current_motor_rpm = 0;

// servo
double last_servo_update_time = 0;
racer::math::angle current_steering_angle = 0;

uint32_t msg_seq;
bool is_initialized;

void command_callback(const geometry_msgs::Twist::ConstPtr &msg) {
  const auto t = ros::Time::now().toSec();

  if (last_servo_update_time > 0) {
    const auto dt = t - last_servo_update_time;
    racer::action action{msg->linear.x, msg->angular.z};
    current_steering_angle = chassis->steering_servo->predict_next_state(
        current_steering_angle, action, dt);
  }

  last_servo_update_time = t;
}

void motor_rpm_callback(const std_msgs::Float64::ConstPtr &msg) {
  current_motor_rpm = msg->data;
}

racer::vehicle_configuration
get_current_configuration(const tf::Transform &transform) {
  auto origin = transform.getOrigin();
  auto rotation = tf::getYaw(transform.getRotation());

  return {origin.x(), origin.y(), rotation};
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "current_state");
  ros::NodeHandle node("~");

  // load parameters
  std::string odom_topic, command_topic, motor_rpm_topic, state_topic;

  node.param<std::string>("command_topic", command_topic, "/racer/commands");
  node.param<std::string>("motor_rpm_topic", motor_rpm_topic,
                          "/racer/motor_rpm");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("map_frame_id", map_frame_id, "map");
  node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  node.param<std::string>("base_link_frame_id", base_link_frame_id,
                          "base_link");

  double frequency;
  node.param<double>("frequency", frequency, 30);

  // reset current state
  msg_seq = 0;
  is_initialized = false;

  // set up pubsub
  ros::Subscriber command_sub =
      node.subscribe<geometry_msgs::Twist>(command_topic, 1, command_callback);
  ros::Subscriber motor_rpm_sub =
      node.subscribe<std_msgs::Float64>(motor_rpm_topic, 1, motor_rpm_callback);
  ros::Publisher state_pub =
      node.advertise<racer_msgs::State>(state_topic, 1, false);

  ros::Rate rate(frequency);
  tf::TransformListener tf_listener;
  ros::Duration speed_averaging_interval(1.0 / double(frequency));
  std::string err;

  ROS_INFO("==> CURRENT STATE NODE is ready to go");
  while (ros::ok()) {
    if (is_initialized) {
      try {
        tf::StampedTransform odom_to_base_link, map_to_odom;
        tf_listener.lookupTransform(odom_frame_id, base_link_frame_id,
                                    ros::Time(0), odom_to_base_link);
        tf_listener.lookupTransform(map_frame_id, odom_frame_id, ros::Time(0),
                                    map_to_odom);

        const auto configuration =
            get_current_configuration(map_to_odom * odom_to_base_link);
        racer::vehicle_model::kinematic::state state{
            configuration, current_motor_rpm, current_steering_angle};

        state_pub.publish(racer_ros::state_to_msg(state, odom_frame_id));
      } catch (tf::TransformException ex) {
        ROS_ERROR("'current_state' node: %s.", ex.what());
      }
    } else if (tf_listener.canTransform(map_frame_id, base_link_frame_id,
                                        ros::Time(0), &err)) {
      is_initialized = true;
    } else {
      err = "";
      ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
