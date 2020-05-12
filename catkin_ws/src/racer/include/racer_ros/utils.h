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

#include "racer/track/occupancy_grid.h"
#include "racer/vehicle/trajectory.h"
#include "racer/vehicle/kinematic/model.h"

using namespace racer::vehicle;

namespace racer_ros
{
  std::unique_ptr<racer::track::occupancy_grid> msg_to_grid(const nav_msgs::OccupancyGrid &map)
  {
    auto data = std::vector<int8_t>{map.data.begin(), map.data.end()};
    racer::math::vector origin{map.info.origin.position.x, map.info.origin.position.y};
    return std::make_unique<racer::track::occupancy_grid>(
        data, map.info.width, map.info.height, map.info.resolution, origin);
  }

  nav_msgs::OccupancyGrid grid_to_msg(const racer::track::occupancy_grid &map)
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

  std::unique_ptr<racer::track::occupancy_grid> load_map(ros::NodeHandle &node)
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

  racer::vehicle::trajectory<kinematic::state> msg_to_trajectory(const racer_msgs::Trajectory &msg, double time_step_s)
  {
    std::vector<racer::vehicle::trajectory_step<kinematic::state>> steps;
  std:
    size_t t = 0;
    for (const auto &step : msg.trajectory)
    {
      racer::vehicle::configuration configuration({step.pose.position.x, step.pose.position.y},
                                                 tf::getYaw(step.pose.orientation));
      const auto motor_rpm = step.motor_rpm.data;
      kinematic::state state{configuration, motor_rpm, 0};

      steps.emplace_back(state, racer::vehicle::action(0, 0), step.next_waypoint.data, t++ * time_step_s);
    }

    return {steps, time_step_s};
  }

  racer_msgs::Trajectory trajectory_to_msg(racer::vehicle::trajectory<kinematic::state> trajectory, std::size_t next_waypoint)
  {
    racer_msgs::Trajectory msg;
    trajectory.header.stamp = ros::Time::now();
    trajectory.header.frame_id = "map";

    for (const auto &step : trajectory.steps()) {
      racer_msgs::TrajectoryState state;

      // the plan only considers the list of waypoints passed to the planner
      // - the first waypoint will have index 0, so its index has to be offset
      // by the actual index of the first waypoint
      state.next_waypoint.data = next_waypoint + step.passed_waypoints();

      state.pose.position.x = step.state().position().x();
      state.pose.position.y = step.state().position().y();
      state.pose.position.z = 0;

      state.pose.orientation = tf::createQuaternionMsgFromYaw(
          step.state().cfg().heading_angle());

      state.motor_rpm.data = step.state().motor_rpm();

      trajectory.trajectory.push_back(state);
    }

    return trajectory;
  }

  racer_msgs::State state_to_msg(const racer::vehicle::kinematic::state &state, std::string odom_frame_id)
  {
    racer_msgs::State msg;
    msg.header.frame_id = odom_frame_id;
    msg.header.stamp = ros::Time::now();

    msg.x = state.position().x();
    msg.y = state.position().y();
    msg.heading_angle = state.cfg().heading_angle();
    msg.motor_rpm = state.motor_rpm();
    msg.steering_angle = state.steering_angle();

    return msg;
  }

  racer::vehicle::kinematic::state msg_to_state(const racer_msgs::State::ConstPtr &msg)
  {
    racer::vehicle::configuration cfg{msg->x, msg->y, msg->heading_angle};
    return {cfg, msg->motor_rpm, msg->steering_angle};
  }

  geometry_msgs::Twist action_to_twist_msg(const racer::vehicle::action &action)
  {
    // twist msgs are used to cotnrol the RC vehicle
    geometry_msgs::Twist twist_msg;
    twist_msg.linear.x = action.throttle();
    twist_msg.angular.z = action.target_steering_angle();
    return twist_msg;
  }

  ackermann_msgs::AckermannDrive action_to_ackermann_msg(const racer::vehicle::action &action,
                                                         const racer::vehicle::steering_servo_model &servo)
  {
    // ackermann msgs are used to control the vehicle in the simulator
    ackermann_msgs::AckermannDrive ackermann_msg;
    ackermann_msg.speed = action.throttle();
    ackermann_msg.steering_angle = action.target_steering_angle();
    return ackermann_msg;
  }

  racer::vehicle::action twist_to_action(const geometry_msgs::Twist::ConstPtr &msg)
  {
    return {msg->linear.x, msg->angular.z};
  }

} // namespace racer_ros
