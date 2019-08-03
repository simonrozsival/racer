#include <iostream>
#include <stdexcept>
#include <ros/ros.h>
#include <vector>
#include <mutex>
#include <sstream>

#include "math/primitives.h"
#include "racing/track_analysis.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"
#include "racing/vehicle_model/base_vehicle_model.h"

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "racer_msgs/Circuit.h"
#include "racer_msgs/Waypoint.h"
#include "racer_msgs/Waypoints.h"

#include "utils.h"

std::mutex odom_lock;
std::mutex analysis_lock;

// settings
int branching_factor;
double max_distance_between_waypoints;
double waypoint_radius, vehicle_radius;
int lookahead;
std::list<math::point> check_points;

// state variables
std::unique_ptr<racing::vehicle_position> position;
std::unique_ptr<racing::occupancy_grid> grid;
std::string frame_id;
std::unique_ptr<std::vector<math::circle>> waypoints;
int next_waypoint = -1;

void load_circuit();

void map_update(const nav_msgs::OccupancyGrid::ConstPtr& map) {
  if (grid) {
    throw std::runtime_error("There is already an existing map.");
  }

  grid = std::make_unique<racing::occupancy_grid>(
    map->data,
    map->info.width,
    map->info.height,
    map->info.resolution,
    math::point(map->info.origin.position.x, map->info.origin.position.y)
  );
  frame_id = map->header.frame_id;

  load_circuit();
}

void load_circuit() {
  if (!position) {
    return;
  }

  if (!grid) {
    return;
  }

  std::cout << "Analyzing the circuit..." << std::endl;

  racing::track_analysis analysis(
    *grid, max_distance_between_waypoints, branching_factor);

  std::list<math::point> final_check_points;
  for (const auto& check_point : check_points) {
    final_check_points.push_back(math::point(check_point.x, check_point.y));
  }
  final_check_points.push_back(position->location()); // back to the start 

  std::cout << "start track analysis/space exploration" << std::endl;
  auto apexes = analysis.find_apexes(vehicle_radius, *position, final_check_points);
  std::cout << "finished track analysis/space exploration" << std::endl;

  if (apexes.size() == 0) {
    std::cout << "cannot find apexes - there might not be enough room for the careful algorithm to fit ghe car through" << std::endl;
    std::cout << "select different checkpoints, decrease the radius of the car, or create a new map" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> guard(analysis_lock);
  std::vector<math::circle> wps;
  for (const auto& apex : apexes) {
    wps.emplace_back(apex, waypoint_radius);
  }

  waypoints = std::make_unique<std::vector<math::circle>>(wps);
  next_waypoint = 0;

  std::cout << "Track analysis is completed. Number of discovered waypoints: " << waypoints->size() << std::endl;
  std::cout << "Next waypoint: " << next_waypoint << std::endl;
}

void odometry_update(const nav_msgs::Odometry::ConstPtr& odom) {
  bool try_init = false;
  if (!position) {
    try_init = true;
  }

  auto state = std::move(msg_to_state(*odom));
  position = std::make_unique<racing::vehicle_position>(state->position);

  if (try_init) {
    load_circuit();
  }

  if (!waypoints) {
    return;
  }

  std::lock_guard<std::mutex> guard(odom_lock);

  if ((*waypoints)[next_waypoint].contains(position->location())) {
    next_waypoint = (next_waypoint + 1) % waypoints->size();
    std::cout << "PASSED A WAYPOINT, next waypoint: " << next_waypoint << std::endl;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "circuit_node");
  ros::NodeHandle node("~");

  std::string map_topic, circuit_topic, odometry_topic, waypoints_topic, waypoints_visualization_topic;

  node.param<std::string>("map_topic", map_topic, "/map");
  node.param<std::string>("odometry_topic", odometry_topic, "/pf/pose/odom");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("waypoints_visualization_topic", waypoints_visualization_topic, "/racer/visualization/waypoints");

  node.param<double>("vehicle_radius", vehicle_radius, 0.3); // m
  node.param<double>("waypoint_radius", waypoint_radius, 1.0); // m

  node.param<double>("max_distance_between_waypoints", max_distance_between_waypoints, 10.0); // m
  node.param<int>("branching_factor", branching_factor, 13);
  node.param<int>("lookahead", lookahead, 3);

  // array of checkpoints
  std::vector<std::string> checkpoint_params;
  node.param<std::vector<std::string>>("check_points", checkpoint_params, std::vector<std::string>());
  for (auto param : checkpoint_params) {
    std::stringstream ss(param);
    double x, y;
    ss >> x;
    ss >> y;
    check_points.emplace_back(x, y);
  }

  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, map_update);
  ros::Subscriber odometry_sub = node.subscribe<nav_msgs::Odometry>(odometry_topic, 1, odometry_update);

  ros::Publisher waypoints_pub = node.advertise<racer_msgs::Waypoints>(waypoints_topic, 1, true);
  ros::Publisher visualization_pub = node.advertise<visualization_msgs::MarkerArray>(waypoints_visualization_topic, 1);

  ros::Rate rate(5);

  while (ros::ok()) {
    std::unique_lock<std::mutex> guard(analysis_lock);
    if (waypoints) {
      racer_msgs::Waypoints msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = frame_id;
      msg.next_waypoint = next_waypoint;

      for (std::size_t i = 0; i < lookahead; ++i) {
        const auto& waypoint = (*waypoints)[(next_waypoint + i) % waypoints->size()];
        racer_msgs::Waypoint wp;
        wp.position.x = waypoint.center.x;
        wp.position.y = waypoint.center.y;
        wp.radius = waypoint.radius;
        
        msg.waypoints.push_back(wp);
      }

      // publish the list of waypoints which the agent should pass next
      waypoints_pub.publish(msg);

      // visualization is published only if somebody is listening
      if (visualization_pub.getNumSubscribers() > 0) {
        visualization_msgs::MarkerArray markers;
        for (std::size_t i = 0; i < waypoints->size(); ++i) {
          bool is_advertised = next_waypoint + lookahead > waypoints->size()
            ? i >= next_waypoint || i < (next_waypoint + lookahead) % waypoints->size()
            : next_waypoint <= i && i < next_waypoint + lookahead;

          const auto wp = (*waypoints)[i];

          visualization_msgs::Marker marker;
          marker.header.frame_id = frame_id;
          marker.header.stamp = ros::Time::now();

          marker.ns = "waypoints";
          marker.id = i;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = wp.center.x;
          marker.pose.position.y = wp.center.y;
          marker.pose.position.z = 0;

          marker.scale.x = 2 * wp.radius;
          marker.scale.y = 2 * wp.radius;
          marker.scale.z = 0.1;

          marker.color.r = is_advertised ? 1.0 : 0.0;
          marker.color.g = 0.0;
          marker.color.b = is_advertised ? 0.0 : 1.0;
          marker.color.a = 0.2;

          markers.markers.push_back(marker);
        }

        visualization_pub.publish(markers);
      }
    }
    guard.unlock();

    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}

