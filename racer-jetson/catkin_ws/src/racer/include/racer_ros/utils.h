#pragma once

#include <iostream>
#include <tf/transform_datatypes.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "racer_msgs/Trajectory.h"
#include "racer_msgs/RacingLine.h"
#include "racer_msgs/RacingLineCorner.h"
#include "racer_msgs/RacingLinePoint.h"

#include "racer/trajectory.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/track/racing_line.h"
#include "racer/splines/catmull_rom.h"

using namespace racer::vehicle_model;

namespace racer_ros
{

std::unique_ptr<racer::occupancy_grid> msg_to_grid(const nav_msgs::OccupancyGrid &map)
{
    auto data = std::vector<uint8_t>{map.data.begin(), map.data.end()};
    // convert signed bytes (which are expected to be in the range 0-100) to unsigned bytes
    return std::make_unique<racer::occupancy_grid>(
        data,
        map.info.width,
        map.info.height,
        map.info.resolution,
        racer::math::vector{map.info.origin.position.x, map.info.origin.position.y});
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
    ROS_ERROR("Cannot obtain the base map from the map service. Another attempt will be made.");
    ros::Duration(1.0).sleep();
    continue;
  }

  return msg_to_grid(map_res.map);
}

kinematic::state pose_and_twist_to_state(
    const geometry_msgs::Pose &pose,
    const geometry_msgs::Twist &twist)
{
    racer::vehicle_configuration configuration(
        {pose.position.x, pose.position.y},
        tf::getYaw(pose.orientation));

    return {
        configuration,
        sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2)),
        0 // we assume the steering angle is always zero in the beginning
    };
}

racer::trajectory<kinematic::state> msg_to_trajectory(
    const racer_msgs::Trajectory &msg, double time_step_s)
{
    std::vector<racer::trajectory_step<kinematic::state>> steps;
    std:size_t t = 0;
    for (const auto &step : msg.trajectory)
    {
        steps.emplace_back(
            pose_and_twist_to_state(step.pose, step.velocity),
            racer::action(0, 0),
            step.next_waypoint.data,
            t++ * time_step_s);
    }

    return {steps, time_step_s};
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

    for (const auto &pt : racing_line.spline()) {
        msg.points.push_back(racing_line_point_msg(pt));
    }

    return msg;
}


racer::track::point msg_to_racing_line_point(const racer_msgs::RacingLinePoint msg)
{
    racer::math::point pt{msg.point.x, msg.point.y};
    return {pt, msg.maximum_speed.data};
}

racer::track::racing_line msg_to_racing_line(const racer_msgs::RacingLine::ConstPtr msg)
{
    std::vector<racer::track::corner> corners;
    for (const auto &corner : msg->corners)
    {
        corners.emplace_back(
            msg_to_racing_line_point(corner.turn_in),
            msg_to_racing_line_point(corner.apex),
            msg_to_racing_line_point(corner.exit));
    }

    std::vector<racer::track::point> spline;
    for (const auto &pt : msg->points) {
        spline.emplace_back(msg_to_racing_line_point(pt));
    }

    return {corners, spline};
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

} // namespace racer_ros
