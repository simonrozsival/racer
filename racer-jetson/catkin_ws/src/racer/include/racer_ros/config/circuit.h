#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>

namespace racer_ros::config {

class circuit {
public:
  const std::string map_topic, circuit_topic, state_topic, waypoints_topic, waypoints_visualization_topic;
  const std::vector<racer::math::point> check_points;
  const int branching_factor;
  const double min_distance_between_waypoints;
  const double waypoint_radius;
  const double vehicle_radius;
  const int lookahead;

private:
  circuit(
    const std::string map_topic,
    const std::string circuit_topic,
    const std::string state_topic,
    const std::string waypoints_topic,
    const std::string waypoints_visualization_topic,
    const std::vector<racer::math::point> check_points,
    const int branching_factor,
    const double min_distance_between_waypoints,
    const double waypoint_radius,
    const double vehicle_radius,
    const int lookahead)
    : map_topic{map_topic},
      circuit_topic{circuit_topic},
      state_topic{state_topic},
      waypoints_topic{waypoints_topic},
      waypoints_visualization_topic{waypoints_visualization_topic},
      check_points{check_points},
      branching_factor{branching_factor},
      min_distance_between_waypoints{min_distance_between_waypoints},
      waypoint_radius{waypoint_radius},
      vehicle_radius{vehicle_radius},
      lookahead{lookahead}
  {
  }

public:
  static circuit load(const ros::NodeHandle& node)
  {
    std::string map_topic, circuit_topic, state_topic, waypoints_topic, waypoints_visualization_topic;
  
    node.param<std::string>("map_topic", map_topic, "/map");
    node.param<std::string>("state_topic", state_topic, "/racer/state");
    node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
    node.param<std::string>("waypoints_visualization_topic", waypoints_visualization_topic, "/racer/visualization/waypoints");

    int branching_factor;
    double min_distance_between_waypoints;
    double waypoint_radius;
    double vehicle_radius;
    int lookahead;
  
    node.param<double>("vehicle_radius", vehicle_radius, 0.25); // m
    node.param<double>("waypoint_radius", waypoint_radius, 1.0); // m
    node.param<double>("min_distance_between_waypoints", min_distance_between_waypoints, 2.0); // m
    node.param<int>("branching_factor", branching_factor, 13);
    node.param<int>("lookahead", lookahead, 3);

    std::vector<std::string> checkpoint_params;
    node.param<std::vector<std::string>>("check_points", checkpoint_params, std::vector<std::string>());

    std::vector<racer::math::point> check_points;
    for (auto param : checkpoint_params)
    {
      std::stringstream ss(param);
      double x, y;
      ss >> x;
      ss >> y;
      check_points.emplace_back(x, y);
    }

    return {
      map_topic,
      circuit_topic,
      state_topic,
      waypoints_topic,
      waypoints_visualization_topic,
      check_points,
      branching_factor,
      min_distance_between_waypoints,
      waypoint_radius,
      vehicle_radius,
      lookahead
    };
  }

};

}