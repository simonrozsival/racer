#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <racer_msgs/State.h>
#include <visualization_msgs/Marker.h>

#include "racer/following_strategies/dwa.h"
#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"

#include "racer_ros/utils.h"

std::optional<racer::vehicle_model::kinematic::state> state;
std::optional<racer::action> action;

void command_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
  action = racer_ros::twist_to_action(msg);
}

void state_callback(const racer_msgs::State::ConstPtr &msg)
{
  state = racer_ros::msg_to_state(msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "visualize_action");
  ros::NodeHandle node("~");

  std::string command_topic, state_topic, visualization_topic;
  node.param<std::string>("command_topic", command_topic, "/racer/commands");
  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/action");

  double time_step_s, prediction_horizon;
  node.param<double>("time_step_s", time_step_s, 1.0 / 20.0);
  node.param<double>("prediction_horizon", prediction_horizon, 0.5);

  auto command_sub = node.subscribe<geometry_msgs::Twist>(command_topic, 1, command_callback);
  auto state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, state_callback);
  auto visualization_pub = node.advertise<visualization_msgs::Marker>(visualization_topic, 100, false);

  const auto model =
      std::make_shared<racer::vehicle_model::kinematic::model>(racer::vehicle_model::vehicle_chassis::rc_beast());
  const racer::following_strategies::unfolder<racer::vehicle_model::kinematic::state> unfolder{
    model, time_step_s, int(std::ceil(prediction_horizon / time_step_s))
  };

  const std::shared_ptr<racer::occupancy_grid> map = racer_ros::load_map(node);

  int seq = 0;

  double frequency = 50.0;
  ros::Rate rate{ frequency };
  while (ros::ok())
  {
    if (state && action)
    {
      const auto prediction = unfolder.unfold(*state, *action, map);

      if (!prediction.empty())
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.seq = seq++;
        marker.header.stamp = ros::Time::now();
        marker.id = 1558;
        marker.lifetime = ros::Duration{ 10 * 1.0 / frequency };
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "action";

        marker.color.g = (action->throttle() + 1) / 2;
        marker.color.r = 1 - marker.color.g;
        marker.color.a = 1.0;

        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;

        for (const auto &s : prediction)
        {
          geometry_msgs::Point pt;
          pt.x = s.position().x();
          pt.y = s.position().y();
          pt.z = 0.0;
          marker.points.push_back(pt);
        }

        visualization_pub.publish(marker);
      }
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
