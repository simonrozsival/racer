#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>

#include <tf/transform_datatypes.h>

#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "racer/TrajectoryMsg.h"

#include "racing-cpp/racing/occupancy_grid_collisions.h"
#include "racing-cpp/racing/kinematic_bicycle_model.h"

std::unique_ptr<racing::occupancy_grid> msg_to_grid(const nav_msgs::OccupancyGrid& map) {  
  return std::make_unique<racing::occupancy_grid>(
    map.data,
    map.info.width,
    map.info.height,
    map.info.resolution,
    math::point(map.info.origin.position.x, map.info.origin.position.y)
  );
}

std::unique_ptr<racing::kinematic_model::state> msg_to_state(
    const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist
) {
    racing::vehicle_position initial_postition(
        pose.position.x,
        pose.position.y,
        tf::getYaw(pose.orientation)
    );

    return std::make_unique<state>(
        initial_postition,
        sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2)),
        0 // we assume the steering angle is always zero in the beginning
    );
}

std::unique_ptr<racing::kinematic_model::state> msg_to_state(const nav_msgs::Odometry& odom) {
    return msg_to_state(odom.pose.pose, odom.twist.twist);
}

std::unique_ptr<racing::kinematic_model::trajectory> msg_to_trajectory(
    const racer::TrajectoryMsg& msg) {
    std::list<racing::kinematic_model::trajectory_step> steps;
    for (const auto& step : msg.trajectory) {
        steps.emplace_back(
            *msg_to_state(step.pose, step.velocity),
            step.next_waypoint.data);
    }

    return std::make_unique<racing::kinematic_model::trajectory>(steps);
}

#endif