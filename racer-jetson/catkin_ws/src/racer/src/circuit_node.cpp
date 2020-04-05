#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "racer/math.h"
#include "racer/occupancy_grid.h"
#include "racer/sehs/space_exploration.h"
#include "racer/track_analysis.h"
#include "racer/vehicle_model/base_model.h"

#include "racer_ros/circuit_progress_monitoring.h"
#include "racer_ros/config/circuit.h"
#include "racer_ros/utils.h"

#include "nav_msgs/OccupancyGrid.h"
#include "racer_msgs/Circuit.h"
#include "racer_msgs/State.h"
#include "racer_msgs/Waypoint.h"
#include "racer_msgs/Waypoints.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

std::size_t last_published_next_waypoint = -1;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "circuit_node");
  ros::NodeHandle node("~");

  auto config = racer_ros::config::circuit::load(node);
  racer_ros::circuit_progress_monitoring circuit{ config };

  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(
      config.map_topic, 1, &racer_ros::circuit_progress_monitoring::map_update, &circuit);
  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(
      config.state_topic, 1, &racer_ros::circuit_progress_monitoring::state_update, &circuit);

  ros::Publisher waypoints_pub = node.advertise<racer_msgs::Waypoints>(config.waypoints_topic, 1, true);
  ros::Publisher visualization_pub =
      node.advertise<visualization_msgs::MarkerArray>(config.waypoints_visualization_topic, 1, true);

  ros::Rate rate(30);

  ROS_INFO("==> CIRCUIT NODE is ready to go");
  while (ros::ok())
  {
    if (circuit.is_initialized())
    {
      const std::size_t next_waypoint = circuit.next_waypoint();
      const auto waypoints_ahead = circuit.waypoints_ahead();

      racer_msgs::Waypoints msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = circuit.frame_id();
      msg.next_waypoint = next_waypoint;

      for (const auto waypoint : waypoints_ahead)
      {
        racer_msgs::Waypoint wp;
        wp.position.x = waypoint.center().x();
        wp.position.y = waypoint.center().y();
        wp.radius = waypoint.radius();

        msg.waypoints.push_back(wp);
      }

      // publish the list of waypoints which the agent should pass next
      waypoints_pub.publish(msg);

      // visualization is published only if somebody is listening
      if (visualization_pub.getNumSubscribers() > 0)
      {
        const auto waypoints = circuit.waypoints();
        const auto lookahead = waypoints_ahead.size();

        std::size_t first_advertised = next_waypoint % waypoints.size();
        std::size_t last_advertised = (next_waypoint + lookahead) % waypoints.size();

        visualization_msgs::MarkerArray markers;
        for (std::size_t i = 0; i < waypoints.size(); ++i)
        {
          bool is_advertised = first_advertised < last_advertised ? first_advertised <= i && i <= last_advertised :
                                                                    first_advertised <= i || i <= last_advertised;

          const auto wp = waypoints[i];

          visualization_msgs::Marker marker;
          marker.header.frame_id = circuit.frame_id();
          marker.header.stamp = ros::Time::now();

          marker.ns = "waypoints";
          marker.id = i;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;

          marker.pose.position.x = wp.center().x();
          marker.pose.position.y = wp.center().y();
          marker.pose.position.z = 0;
          marker.pose.orientation.w = 1.0;

          marker.scale.x = 2 * wp.radius();
          marker.scale.y = 2 * wp.radius();
          marker.scale.z = 0.1;

          marker.color.r = is_advertised ? 1.0 : 0.0;
          marker.color.g = 0.0;
          marker.color.b = is_advertised ? 0.0 : 1.0;
          marker.color.a = 0.2;

          markers.markers.push_back(marker);
        }

        visualization_pub.publish(markers);
      }

      last_published_next_waypoint = next_waypoint;
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
