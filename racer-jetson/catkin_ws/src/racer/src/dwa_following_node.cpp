#include <stdexcept>

#include <ros/ros.h>
#include <cmath>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

#include <racer_msgs/State.h>
#include <racer_msgs/Trajectory.h>
#include <racer_msgs/Waypoints.h>

#include <dynamic_reconfigure/server.h>
#include <racer/DWAConfig.h>

#include "racing/vehicle_model/kinematic_bicycle_model.h"
#include "racing/vehicle_model/base_vehicle_model.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"
#include "racing/following_strategies/dwa.h"
#include "math/euler_method_integrator.h"
#include "Follower.h"

std::shared_ptr<racing::dwa> dwa;
std::shared_ptr<racing::trajectory_error_calculator> error_calculator;

visualization_msgs::MarkerArray prepare_visualization(
  const Follower& follower,
  const racing::kinematic_model::action& selected_action,
  const std::list<racing::kinematic_model::action>& all_actions);

void spin(
  const int frequency,
  const Follower& follower,
  const std::list<racing::kinematic_model::action> actions,
  const double max_allowed_speed_percentage,
  const ros::Publisher command_pub,
  const ros::Publisher visualization_pub) {

  ros::Rate rate(frequency);

  while (ros::ok()) {
    if (follower.is_initialized()) {
      auto action = follower.select_driving_command();

      if (!action) {
        action = follower.stop();
        ROS_DEBUG("following node: STOP!");
      } else {
        ROS_DEBUG("following node selected: [throttle: %f, steering angle: %f]", action->throttle, action->target_steering_angle);
      }

      geometry_msgs::Twist msg;
      msg.linear.x = max_allowed_speed_percentage * action->throttle;
      msg.angular.z = -action->target_steering_angle;

      command_pub.publish(msg);

      if (visualization_pub.getNumSubscribers() > 0) {
        const auto msg = prepare_visualization(
          follower,
          *action,
          actions);
        visualization_pub.publish(msg);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
}

void dynamic_reconfigure_callback(const racer::DWAConfig& config, uint32_t level) {
  if (!dwa) {
    ROS_ERROR("Cannot handle dynamic reconfiguration because the initialization of this node hasn't finished yet.");
    return;
  }

  error_calculator = std::make_shared<racing::trajectory_error_calculator>(
    config.position_weight,
    config.heading_weight,
    config.velocity_weight,
    config.velocity_undershoot_overshoot_ratio,
    config.distance_to_obstacle_weight,
    config.max_position_error);

  dwa->reconfigure(error_calculator);
  ROS_INFO("DWA was reconfigured");
}

void setup_dynamic_reconfigure() {
  dynamic_reconfigure::Server<racer::DWAConfig> server;
  dynamic_reconfigure::Server<racer::DWAConfig>::CallbackType f;  
  f = boost::bind(&dynamic_reconfigure_callback, _1, _2);
  server.setCallback(f);
  ROS_INFO("dynamic reconfiguration for DWA was set up");
}

void create_visualization_line(
  int id,
  const std::list<racing::kinematic_model::state>& trajectory,
  double score,
  visualization_msgs::Marker& line) {

  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.scale.x = 0.01;
  line.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.ns = "dwa";
  line.id = id;

  line.color.r = score;
  line.color.g = 1.0 - score; // the lower, the better

  line.color.a = 0.5;

  for (const auto state : trajectory) {
    geometry_msgs::Point point;

    point.x = state.position.x;
    point.y = state.position.y;
    point.z = 0;

    line.points.push_back(point);
  }
}

visualization_msgs::MarkerArray prepare_visualization(
  const Follower& follower,
  const racing::kinematic_model::action& selected_action,
  const std::list<racing::kinematic_model::action>& all_actions) {

  visualization_msgs::MarkerArray msg;

  const auto from = follower.last_known_state();
  const auto costmap = follower.costmap();

  const auto reference_subtrajectory = follower.reference_trajectory().find_reference_subtrajectory(from, follower.next_waypoint());
  if (reference_subtrajectory == nullptr) {
    return msg;
  }

  int id = 0;
  for (const auto action : all_actions) {
    const auto trajectory = dwa->unfold(from, action, costmap);
    if (trajectory) {
      visualization_msgs::Marker line;
      line.header.frame_id = follower.map_frame_id;
      line.header.stamp = ros::Time::now();

      const double score = error_calculator->calculate_error(*trajectory, *reference_subtrajectory, costmap);
      create_visualization_line(id++, *trajectory, score, line);

      msg.markers.push_back(line);
    }
  }

  const auto trajectory = dwa->unfold(from, selected_action, costmap);
  if (trajectory) {
    visualization_msgs::Marker selected_line;
    selected_line.header.frame_id = follower.map_frame_id;
    selected_line.header.stamp = ros::Time::now();

    const double score = error_calculator->calculate_error(*trajectory, *reference_subtrajectory, costmap);
    create_visualization_line(id++, *trajectory, score, selected_line);
 
    selected_line.scale.x = 0.05;
    selected_line.color.a = 1.0;
    
    msg.markers.push_back(selected_line);
  }

  return msg;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "dwa_following_node");
  ros::NodeHandle node("~");

  double cell_size;
  std::string state_topic, trajectory_topic, waypoints_topic, costmap_topic, driving_topic, visualization_topic;

  node.param<double>("double", cell_size, 0.05);

  node.param<std::string>("costmap_topic", costmap_topic, "/costmap");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");
  node.param<std::string>("state_topic", state_topic, "/racer/state");

  node.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/dwa");

  double max_allowed_speed_percentage;
  double max_speed, max_reversing_speed, acceleration;

  node.param<double>("max_allowed_speed_percentage", max_allowed_speed_percentage, 1.0);
  node.param<double>("max_speed", max_speed, 6.0);
  node.param<double>("max_reversing_speed", max_reversing_speed, -3.0);
  node.param<double>("acceleration", acceleration, 3.0);

  racing::vehicle vehicle(
    0.155, // cog_offset
    0.31, // wheelbase
    0.35, // safe width
    0.55, // safe length
    2.0 / 3.0 * M_PI, // steering speed (rad/s)
    1.0 / 6.0 * M_PI, // max steering angle (rad)
    max_allowed_speed_percentage * max_speed, // speed (ms^-1)
    max_allowed_speed_percentage * max_reversing_speed, // speed (ms^-1)
    acceleration // acceleration (ms^-2)
  );

  double integration_step_s, prediction_horizon_s;
  node.param<double>("integration_step_s", integration_step_s, 1.0 / 20.0);
  node.param<double>("prediction_horizon_s", prediction_horizon_s, 0.5);

  std::shared_ptr<racing::kinematic_model::model> model =
    std::make_shared<racing::kinematic_model::model>(std::make_unique<math::euler_method>(integration_step_s), vehicle);

  const int lookahead = int(ceil(prediction_horizon_s / integration_step_s));

  ROS_DEBUG("DWA strategy");
  auto actions = racing::kinematic_model::action::create_actions_including_reverse(9, 15);

  double position_weight, heading_weight, velocity_weight, distance_to_obstacle_weight;
  node.param<double>("position_weight", position_weight, 30.0);
  node.param<double>("heading_weight", heading_weight, 20.0);
  node.param<double>("velocity_weight", velocity_weight, 10.0);
  node.param<double>("distance_to_obstacle_weight", distance_to_obstacle_weight, 5.0);

  error_calculator = 
    std::make_shared<racing::trajectory_error_calculator>(
      position_weight,
      heading_weight,
      velocity_weight,
      1.0,
      distance_to_obstacle_weight,
      vehicle.radius() * 5
    );

  dwa =
    std::make_shared<racing::dwa>(
      lookahead,
      actions,
      model,
      error_calculator
    );

  ROS_DEBUG("DWA following strategy was initialized");
  Follower follower(dwa);

  setup_dynamic_reconfigure();
 
  ros::Subscriber costmap_sub = node.subscribe<nav_msgs::OccupancyGrid>(costmap_topic, 1, &Follower::costmap_observed, &follower);
  ros::Subscriber trajectory_sub = node.subscribe<racer_msgs::Trajectory>(trajectory_topic, 1, &Follower::trajectory_observed, &follower);
  ros::Subscriber waypoints_sub = node.subscribe<racer_msgs::Waypoints>(waypoints_topic, 1, &Follower::waypoints_observed, &follower);
  ros::Subscriber state_sub = node.subscribe<racer_msgs::State>(state_topic, 1, &Follower::state_observed, &follower);

  ros::Publisher command_pub = node.advertise<geometry_msgs::Twist>(driving_topic, 1);
  ros::Publisher visualization_pub = node.advertise<visualization_msgs::MarkerArray>(visualization_topic, 1, true);

  int frequency; // Hz
  node.param<int>("update_frequency_hz", frequency, 30);

  spin(frequency, follower, actions, max_allowed_speed_percentage, command_pub, visualization_pub);

  return 0;
}
