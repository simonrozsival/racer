#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>

#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "racer_msgs/Trajectory.h"

#include "racer/occupancy_grid.h"
#include "racer/vehicle_model/kinematic_bicycle_model.h"

using namespace racer;

namespace racer_ros {

    std::unique_ptr<racer::occupancy_grid> msg_to_grid(const nav_msgs::OccupancyGrid& map) {
        // convert signed bytes (which are expected to be in the range 0-100) to unsigned bytes
        std::vector<uint8_t> data { map.data.begin(), map.data.end() };
        return std::make_unique<racer::occupancy_grid>(
            data,
            map.info.width,
            map.info.height,
            map.info.resolution,
            racer::math::point(map.info.origin.position.x, map.info.origin.position.y)
        );
    }

    std::unique_ptr<racer::vehicle_model::kinematic_bicycle_model::state> pose_and_twist_to_state(
        const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist
    ) {
        racer::vehicle_position initial_postition(
            pose.position.x,
            pose.position.y,
            tf::getYaw(pose.orientation)
        );

        return std::make_unique<racer::vehicle_model::kinematic_bicycle_model::state>(
            initial_postition,
            sqrt(pow(twist.linear.x, 2) + pow(twist.linear.y, 2)),
            0 // we assume the steering angle is always zero in the beginning
        );
    }

    std::unique_ptr<racer::vehicle_model::kinematic_bicycle_model::trajectory> msg_to_trajectory(
        const racer_msgs::Trajectory& msg) {
        std::list<racer::vehicle_model::kinematic_bicycle_model::trajectory_step> steps;
        for (const auto& step : msg.trajectory) {
            steps.emplace_back(
                *pose_and_twist_to_state(step.pose, step.velocity),
                step.next_waypoint.data);
        }

        return std::make_unique<racer::vehicle_model::kinematic_bicycle_model::trajectory>(steps);
    }
}

#endif
