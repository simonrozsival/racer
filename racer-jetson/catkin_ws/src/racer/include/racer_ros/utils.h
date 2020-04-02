#pragma once

#include <iostream>
#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "racer_msgs/Trajectory.h"
#include "racer_msgs/RacingLine.h"
#include "racer_msgs/RacingLineCorner.h"

#include "racer/trajectory.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/track/racing_line.h"
#include "racer/splines/catmull_rom.h"

using namespace racer::vehicle_model;

namespace racer_ros
{

std::shared_ptr<racer::occupancy_grid> msg_to_grid(const nav_msgs::OccupancyGrid &map)
{
    auto data = std::vector<uint8_t>{map.data.begin(), map.data.end()};
    // convert signed bytes (which are expected to be in the range 0-100) to unsigned bytes
    return std::make_shared<racer::occupancy_grid>(
        data,
        map.info.width,
        map.info.height,
        map.info.resolution,
        racer::math::vector{map.info.origin.position.x, map.info.origin.position.y});
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

racer_msgs::RacingLine racing_line_to_msg(const racer::track::racing_line &racing_line)
{
    racer_msgs::RacingLine msg;
    
    for (const auto &corner : racing_line.corners)
    {
        racer_msgs::RacingLineCorner corner_msg;

        corner_msg.turn_in.point.x = corner.turn_in.grid_coordinate.x();
        corner_msg.turn_in.point.y = corner.turn_in.grid_coordinate.y();
        corner_msg.turn_in.maximum_speed.data = corner.turn_in.maximum_speed;

        corner_msg.apex.point.x = corner.apex.grid_coordinate.x();
        corner_msg.apex.point.y = corner.apex.grid_coordinate.y();
        corner_msg.apex.maximum_speed.data = corner.apex.maximum_speed;

        corner_msg.exit.point.x = corner.exit.grid_coordinate.x();
        corner_msg.exit.point.y = corner.exit.grid_coordinate.y();
        corner_msg.exit.maximum_speed.data = corner.exit.maximum_speed;

        msg.corners.push_back(corner_msg);
    }

    std::vector<racer::math::point> control_points;
    for (const auto &corner : racing_line.corners) {
        control_points.push_back(corner.turn_in.grid_coordinate);
        control_points.push_back(corner.apex.grid_coordinate);
        control_points.push_back(corner.exit.grid_coordinate);
    }

    const auto points = racer::splines::catmull_rom::enumerate_loop(control_points, 0.1);
    for (const auto &pt : points) {
        geometry_msgs::Point pt_msg;
        pt_msg.x = pt.x();
        pt_msg.y = pt.y();
        msg.points.push_back(pt_msg);
    }

    return msg;
}


} // namespace racer_ros
