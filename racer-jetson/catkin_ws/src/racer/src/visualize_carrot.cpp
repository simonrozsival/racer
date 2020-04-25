#include <iostream>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <racer_msgs/State.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>
#include <visualization_msgs/Marker.h>

#include "racer/following_strategies/dwa_strategy.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/following_strategies/target_locator.h"

#include "racer_ros/utils.h"

std::optional<racer::vehicle_model::kinematic::state> state;
std::optional<racer::trajectory<racer::vehicle_model::kinematic::state>> trajectory;
std::size_t next_waypoint;

void trajectory_callback(const racer_msgs::Trajectory::ConstPtr &msg)
{
  trajectory = racer_ros::msg_to_trajectory(*msg, 0.02);
}

void state_callback(const racer_msgs::State::ConstPtr &msg)
{
  state = racer_ros::msg_to_state(msg);
}

void waypoints_observed(const racer_msgs::Waypoints::ConstPtr &waypoints)
{
  next_waypoint = waypoints->next_waypoint;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "visualize_action");
  ros::NodeHandle node("~");

  std::string trajectory_topic, state_topic, visualization_topic, waypoints_topic;
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/carrot");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");

  double min_lookahead, max_lookahead;
  node.param<double>("min_lookahead", min_lookahead, 1.0);
  node.param<double>("max_lookahead", max_lookahead, 2.0);

  auto trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, trajectory_callback);
  auto state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, state_callback);
  auto visualization_pub = node.advertise<visualization_msgs::Marker>(visualization_topic, 100, false);
  auto waypoints_sub =
      node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, waypoints_observed);

  auto model =
      std::make_unique<racer::vehicle_model::kinematic::model>(racer::vehicle_model::vehicle_chassis::simulator());
  racer::following_strategies::target_locator<racer::vehicle_model::kinematic::state> target_locator{
      min_lookahead, max_lookahead, model->chassis->motor->max_rpm()};

  int seq = 0;

  double frequency = 100.0;
  ros::Rate rate{frequency};
  while (ros::ok())
  {
    if (state && trajectory)
    {
      const auto target = target_locator.find_target(*state, next_waypoint, *trajectory);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.seq = seq++;
      marker.header.stamp = ros::Time::now();
      marker.id = 1559;
      marker.lifetime = ros::Duration{10 * 1.0 / frequency};
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.ns = "carrot";

      marker.color.r = 1.0;
      marker.color.a = 1.0;
      marker.scale.x = 0.25;
      marker.scale.y = 0.25;
      marker.scale.z = 0.25;

      marker.pose.position.x = target.position().x();
      marker.pose.position.y = target.position().y();
      marker.pose.orientation.w = 1.0;

      visualization_pub.publish(marker);
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
