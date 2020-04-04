#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <racer_msgs/State.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "racer/following_strategies/dwa.h"
#include "racer/math.h"
#include "racer/vehicle_configuration.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/vehicle_model/motor_model.h"
#include "racer/vehicle_model/steering_servo_model.h"

#include "racer_ros/utils.h"

using namespace racer::vehicle_model;

std::optional<racer::trajectory<kinematic::state>> trajectory;

void trajectory_callback(const racer_msgs::Trajectory::ConstPtr &msg)
{
  const double time_step_s = 0.1;
  trajectory = racer_ros::msg_to_trajectory(*msg, time_step_s);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "visualize_plan");
  ros::NodeHandle node("~");

  std::string trajectory_topic, state_topic, visualization_topic;
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/trajectory");

  auto trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, trajectory_callback);
  auto visualization_pub = node.advertise<visualization_msgs::MarkerArray>(visualization_topic, 1, true);

  const auto model = std::make_shared<kinematic::model>(vehicle_chassis::rc_beast());
  int seq = 0;

  double frequency = 5.0;
  ros::Rate rate{ frequency };

  while (ros::ok())
  {
    if (trajectory)
    {
      visualization_msgs::MarkerArray markers;

      for (const auto &step : trajectory->steps())
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.id = seq;
        marker.header.seq = seq++;
        marker.header.stamp = ros::Time::now();
        marker.lifetime = ros::Duration{ 1 / frequency };
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "trajectory";

        double speed = model->calculate_speed_with_no_slip_assumption(step.state().motor_rpm()) /
                       model->maximum_theoretical_speed();
        marker.color.g = speed;
        marker.color.r = 1 - speed;
        marker.color.a = 1.0;

        marker.pose.position.x = step.state().position().x();
        marker.pose.position.y = step.state().position().y();

        double heading = step.state().configuration().heading_angle();
        marker.pose.orientation.w = cos(0.5 * heading);
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = sin(0.5 * heading);

        marker.scale.x = 0.2 + 0.5 * speed;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        markers.markers.push_back(marker);
      }

      visualization_pub.publish(markers);
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
