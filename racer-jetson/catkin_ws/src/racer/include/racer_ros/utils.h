#pragma once

#include <iostream>
#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "racer_msgs/Trajectory.h"

#include "racer/trajectory.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_model/kinematic_model.h"

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
            step.next_waypoint.data,
            t++ * time_step_s);
    }

    return {steps, time_step_s};
}

} // namespace racer_ros
