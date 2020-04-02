#include <iostream>
#include <vector>

#include <ros/ros.h>

#include "racer/math.h"
#include "racer/track_analysis.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_model/base_model.h"

#include "racer_ros/utils.h"
#include "racer_ros/config/circuit.h"
#include "racer_ros/circuit_progress_monitoring.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "racer_msgs/State.h"
#include "racer_msgs/RacingLine.h"

std::shared_ptr<racer::vehicle_model::kinematic::model> model;
std::shared_ptr<racer::vehicle_model::vehicle_chassis> vehicle;

std::shared_ptr<racer::occupancy_grid> grid;
std::vector<racer::math::point> check_points;

std::unique_ptr<racer::track::racing_line> racing_line;
bool new_line = false;

void load_map(ros::NodeHandle &node)
{
  // get the base map for space exploration
  while (!ros::service::waitForService("static_map", ros::Duration(3.0)))
  {
    ROS_INFO("'racing_line_planning_node': Map service isn't available yet.");
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

  grid = racer_ros::msg_to_grid(map_res.map);
}

void state_update(const racer_msgs::State::ConstPtr msg)
{
  // this must be run only once
  if (!grid || racing_line) return;

  racer::vehicle_configuration current_configuration{msg->x, msg->y, msg->heading_angle};

  // the circuit is defined by the initial (current) configuration of the vehicle and the given
  // checkpoints along the track
  std::vector<racer::math::point> final_check_points{check_points.begin(), check_points.end()};
  final_check_points.push_back(current_configuration.location()); // back to the start

  // calculate the centerline of the track in the occupancy grid and detect the corners along the centerline
  const auto centerline = racer::track::centerline::find(current_configuration, grid, final_check_points);  
  racer::track_analysis analysis{centerline.width()};
  const auto pivot_points = analysis.find_pivot_points(centerline.circles(), final_check_points, grid);
  const auto corners = analysis.find_corners(pivot_points, final_check_points, M_PI * 4.0 / 5.0);
  if (corners.empty())
  {
    ROS_ERROR("cannot find any corners of the track");
    ROS_DEBUG("select different checkpoints, decrease the radius of the car, or create a new map");
    return;
  }

  // create initial trivial line
  const auto maximum_speed = model->maximum_theoretical_speed();
  racing_line = racer::track::racing_line::construct_trivial(centerline, corners, maximum_speed);
  new_line = true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "racing_line_planning_node");
  ros::NodeHandle node("~");
  
  std::string state_topic, racing_line_topic;
  node.param<std::string>("state_topic", state_topic, "/racer/state");
  node.param<std::string>("racing_line_topic", racing_line_topic, "/racer/racing_line");

  std::vector<std::string> checkpoint_params;
  node.param<std::vector<std::string>>("check_points", checkpoint_params, std::vector<std::string>());

  for (auto param : checkpoint_params)
  {
    std::stringstream ss(param);
    double x, y;
    ss >> x;
    ss >> y;
    check_points.emplace_back(x, y);
  }

  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, &state_update);
  ros::Publisher racing_line_pub = node.advertise<racer_msgs::RacingLine>(racing_line_topic, 1, true);

  vehicle = racer::vehicle_model::vehicle_chassis::rc_beast();
  model = std::make_shared<racer::vehicle_model::kinematic::model>(vehicle);

  // this will block until we get the occupancy grid from the service
  load_map(node);

  ros::Rate publish_frequency{1.0};
  while (ros::ok())
  {
    if (racing_line && new_line) {
      new_line = false;
      const auto msg = racer_ros::racing_line_to_msg(*racing_line);
      racing_line_pub.publish(msg);
    }

    ros::spinOnce();
    publish_frequency.sleep();
  }

  return 0;
}
