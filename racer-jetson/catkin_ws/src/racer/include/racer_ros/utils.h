#pragma once

#include <tf/transform_datatypes.h>
#include <iostream>

#include "ackermann_msgs/AckermannDrive.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

#include "racer_msgs/RacingLine.h"
#include "racer_msgs/RacingLineCorner.h"
#include "racer_msgs/RacingLinePoint.h"
#include "racer_msgs/State.h"
#include "racer_msgs/Trajectory.h"

#include "racer/occupancy_grid.h"
#include "racer/splines/catmull_rom.h"
#include "racer/track/racing_line.h"
#include "racer/trajectory.h"
#include "racer/vehicle_model/kinematic_model.h"

using namespace racer::vehicle_model;

namespace racer_ros
{
std::unique_ptr<racer::occupancy_grid> msg_to_grid(const nav_msgs::OccupancyGrid &map)
{
  auto data = std::vector<int8_t>{ map.data.begin(), map.data.end() };
  // convert signed bytes (which are expected to be in the range 0-100) to
  // unsigned bytes
  return std::make_unique<racer::occupancy_grid>(
      data, map.info.width, map.info.height, map.info.resolution,
      racer::math::vector{ map.info.origin.position.x, map.info.origin.position.y });
}

nav_msgs::OccupancyGrid grid_to_msg(const racer::occupancy_grid &map)
{
  nav_msgs::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.width = map.cols();
  msg.info.height = map.rows();
  msg.info.resolution = map.cell_size();
  msg.info.origin.position.x = map.origin().x();
  msg.info.origin.position.y = map.origin().y();
  msg.data = map.raw_data();
  return msg;
}

std::unique_ptr<racer::occupancy_grid> load_map(ros::NodeHandle &node)
{
  // get the base map for space exploration
  while (!ros::service::waitForService("static_map", ros::Duration(3.0)))
  {
    ROS_INFO("Map service isn't available yet.");
    continue;
  }

  auto map_service_client = node.serviceClient<nav_msgs::GetMap>("/static_map");

  nav_msgs::GetMap::Request map_req;
  nav_msgs::GetMap::Response map_res;
  while (!map_service_client.call(map_req, map_res))
  {
    ROS_ERROR("Cannot obtain the base map from the map service. Another "
              "attempt will be made.");
    ros::Duration(1.0).sleep();
    continue;
  }

  return msg_to_grid(map_res.map);
}

racer::trajectory<kinematic::state> msg_to_trajectory(const racer_msgs::Trajectory &msg, double time_step_s)
{
  std::vector<racer::trajectory_step<kinematic::state>> steps;
std:
  size_t t = 0;
  for (const auto &step : msg.trajectory)
  {
    racer::vehicle_configuration configuration({ step.pose.position.x, step.pose.position.y },
                                               tf::getYaw(step.pose.orientation));
    const auto motor_rpm = step.motor_rpm.data;
    kinematic::state state{ configuration, motor_rpm, 0 };

    steps.emplace_back(state, racer::action(0, 0), step.next_waypoint.data, t++ * time_step_s);
  }

  return { steps, time_step_s };
}

racer_msgs::State state_to_msg(const racer::vehicle_model::kinematic::state &state, std::string odom_frame_id)
{
  racer_msgs::State msg;
  msg.header.frame_id = odom_frame_id;
  msg.header.stamp = ros::Time::now();

  msg.x = state.position().x();
  msg.y = state.position().y();
  msg.heading_angle = state.configuration().heading_angle();
  msg.motor_rpm = state.motor_rpm();
  msg.steering_angle = state.steering_angle();

  return msg;
}

racer::vehicle_model::kinematic::state msg_to_state(const racer_msgs::State::ConstPtr &msg)
{
  racer::vehicle_configuration cfg{ msg->x, msg->y, msg->heading_angle };
  return { cfg, msg->motor_rpm, msg->steering_angle };
}

racer_msgs::RacingLinePoint racing_line_point_msg(const racer::track::point &pt)
{
  racer_msgs::RacingLinePoint pt_msg;
  pt_msg.point.x = pt.coordinate.x();
  pt_msg.point.y = pt.coordinate.y();
  pt_msg.maximum_speed.data = pt.maximum_speed;
  return pt_msg;
}

racer_msgs::RacingLine racing_line_to_msg(const racer::track::racing_line &racing_line)
{
  racer_msgs::RacingLine msg;

  for (const auto &corner : racing_line.corners())
  {
    racer_msgs::RacingLineCorner corner_msg;
    corner_msg.turn_in = racing_line_point_msg(corner.turn_in);
    corner_msg.apex = racing_line_point_msg(corner.apex);
    corner_msg.exit = racing_line_point_msg(corner.exit);
    msg.corners.push_back(corner_msg);
  }

  for (const auto &pt : racing_line.spline())
  {
    msg.points.push_back(racing_line_point_msg(pt));
  }

  return msg;
}

racer::track::point msg_to_racing_line_point(const racer_msgs::RacingLinePoint msg)
{
  racer::math::point pt{ msg.point.x, msg.point.y };
  return { pt, msg.maximum_speed.data };
}

racer::track::racing_line msg_to_racing_line(const racer_msgs::RacingLine::ConstPtr msg)
{
  std::vector<racer::track::corner> corners;
  for (const auto &corner : msg->corners)
  {
    corners.emplace_back(msg_to_racing_line_point(corner.turn_in), msg_to_racing_line_point(corner.apex),
                         msg_to_racing_line_point(corner.exit));
  }

  std::vector<racer::track::point> spline;
  for (const auto &pt : msg->points)
  {
    spline.emplace_back(msg_to_racing_line_point(pt));
  }

  return { corners, spline };
}

nav_msgs::Path visualize_trajectory(racer_msgs::Trajectory plan)
{
  nav_msgs::Path path;
  path.header = plan.header;

  for (const auto &step : plan.trajectory)
  {
    geometry_msgs::PoseStamped path_pose;
    path_pose.header = plan.header;
    path_pose.pose = step.pose;

    path.poses.push_back(path_pose);
  }

  return path;
}

geometry_msgs::PoseStamped pose_msg_from_point(racer::math::point pt)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = pt.x();
  pose.pose.position.y = pt.y();
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::Twist action_to_twist_msg(const racer::action &action)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = action.throttle();
  twist_msg.angular.z = action.target_steering_angle();
  return twist_msg;
}

ackermann_msgs::AckermannDrive action_to_ackermann_msg(const racer::action &action)
{
  ackermann_msgs::AckermannDrive ackermann_msg;
  ackermann_msg.speed = action.throttle();
  ackermann_msg.steering_angle = action.target_steering_angle();
  return ackermann_msg;
}

racer::action twist_to_action(const geometry_msgs::Twist::ConstPtr &msg)
{
  return { msg->linear.x, msg->angular.z };
}

}  // namespace racer_ros
