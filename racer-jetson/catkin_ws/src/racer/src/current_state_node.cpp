#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>
#include <racer_msgs/State.h>

#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"
#include "racer/vehicle_model/kinematic_model.h"

// params
std::string map_frame_id, odom_frame_id, base_link_frame_id;
double shaft_to_motor_ratio;

// motor
double last_rpm_update_time = 0;
double revolutions = 0;
double current_motor_rpm = 0;

// servo
double last_servo_update_time = 0;
racer::math::angle current_steering_angle = 0;
auto servo = racer::vehicle_model::steering_servo_model::with_fitted_values();

uint32_t msg_seq;
bool is_initialized;

void command_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  const auto t = ros::Time::now().toSec();

  if (last_servo_update_time > 0)
  {
    const auto dt = t - last_servo_update_time;
    racer::action action{msg->linear.x, msg->angular.z};
    current_steering_angle = servo->predict_next_state(current_steering_angle, action, dt);
  }

  last_servo_update_time = t;
}

void wheel_encoder_callback(const std_msgs::Float64::ConstPtr &msg)
{
  const double revs = msg->data * shaft_to_motor_ratio;
  const double t = ros::Time::now().toSec();

  if (last_rpm_update_time > 0)
  {
    const auto dt = t - last_rpm_update_time;
    const auto drevs = revs - revolutions;
    const auto revs_per_second = drevs / dt;
    current_motor_rpm = revs_per_second * 60;
  }

  revolutions = revs;
  last_rpm_update_time = t;
}

racer::vehicle_configuration get_current_configuration(const tf::Transform &transform)
{
  auto origin = transform.getOrigin();
  auto rotation = tf::getYaw(transform.getRotation());

  return {origin.x(), origin.y(), rotation};
}

void publish_state(
    const ros::Publisher &state_pub,
    const racer::vehicle_configuration &configuration,
    const double current_rpm,
    const double current_steering_angle)
{
  racer_msgs::State state;
  state.header.seq = msg_seq++;
  state.header.frame_id = odom_frame_id;
  state.header.stamp = ros::Time::now();

  state.x = configuration.location().x();
  state.y = configuration.location().y();
  state.heading_angle = configuration.heading_angle();
  state.motor_rpm = current_motor_rpm;
  state.steering_angle = current_steering_angle;

  state_pub.publish(state);
}

void spin(const int frequency, const ros::Publisher &state_pub)
{
  ros::Rate rate(frequency);
  tf::TransformListener tf_listener;
  ros::Duration speed_averaging_interval(1.0 / double(frequency));
  std::string err;

  while (ros::ok())
  {
    if (is_initialized)
    {
      try
      {
        tf::StampedTransform odom_to_base_link, map_to_odom;
        tf_listener.lookupTransform(odom_frame_id, base_link_frame_id, ros::Time(0), odom_to_base_link);
        tf_listener.lookupTransform(map_frame_id, odom_frame_id, ros::Time(0), map_to_odom);

        const auto configuration = get_current_configuration(map_to_odom * odom_to_base_link);
        publish_state(state_pub, configuration, current_motor_rpm, current_steering_angle);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("'current_state' node: %s.", ex.what());
      }
    }
    else if (tf_listener.canTransform(map_frame_id, base_link_frame_id, ros::Time(0), &err))
    {
      is_initialized = true;
      ROS_INFO("'current_state' node: INITIALIZED.");
    }
    else
    {
      ROS_INFO("'current_state' node: Still not initialized (%s).", err.c_str());
      err = "";
      ros::Duration(1.0).sleep();
    }

    ros::spinOnce();
    rate.sleep();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "current_state");
  ros::NodeHandle node("~");

  // load parameters
  std::string odom_topic, command_topic, wheel_encoder_topic, state_topic;

  node.param<std::string>("command_topic", command_topic, "/racer/commands");
  node.param<std::string>("wheel_encoder_topic", wheel_encoder_topic, "/racer/wheel_encoders");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("map_frame_id", map_frame_id, "map");
  node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  node.param<std::string>("base_link_frame_id", base_link_frame_id, "base_link");

  node.param<double>("shaft_to_motor_ratio", shaft_to_motor_ratio, 3.4);

  int frequency;
  node.param<int>("frequency", frequency, 30);

  // reset current state
  msg_seq = 0;
  is_initialized = false;

  // set up pubsub
  ros::Subscriber command_sub = node.subscribe<geometry_msgs::Twist>(command_topic, 1, command_callback);
  ros::Subscriber wheel_encoder_sub = node.subscribe<std_msgs::Float64>(wheel_encoder_topic, 1, wheel_encoder_callback);
  ros::Publisher state_pub = node.advertise<racer_msgs::State>(state_topic, 1, false);

  spin(frequency, state_pub);

  return 0;
}
