#include "ros/ros.h"
#include <cmath>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include "racer/TrajectoryMsg.h"
#include "racer/WaypointsMsg.h"

#include "racing/kinematic_bicycle_model.h"
#include "racing/base_vehicle_model.h"
#include "racing/occupancy_grid_collisions.h"
#include "racing/dwa.h"
#include "math/euler_method_integrator.h"
#include "Follower.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "following_node");
  ros::NodeHandle node;

  double cell_size;
  std::string odometry_topic, trajectory_topic, waypoints_topic, map_topic, driving_topic, visualization_topic;

  node.param<double>("double", cell_size, 0.05);

  node.param<std::string>("map_topic", map_topic, "/map");
  node.param<std::string>("odometry_topic", odometry_topic, "/pf/pose/odom");
  node.param<std::string>("trajectory_topic", trajectory_topic, "/racer/trajectory");
  node.param<std::string>("waypoints_topic", waypoints_topic, "/racer/waypoints");

  node.param<std::string>("driving_topic", driving_topic, "/racer/commands");
  node.param<std::string>("visualization_topic", visualization_topic, "/racer/visualization/dwa");

  double max_speed, acceleration;

  node.param<double>("vehicle_max_speed", max_speed, 3.0);
  node.param<double>("vehicle_acceleration", acceleration, 2.0);

  racing::vehicle_properties vehicle(
    0.155, // cog_offset
    0.31, // wheelbase
    0.35, // safe width
    0.55, // safe length
    2.0 / 3.0 * M_PI, // steering speed (rad/s)
    1.0 / 6.0 * M_PI, // max steering angle (rad)
    max_speed, // speed (ms^-1)
    acceleration // acceleration (ms^-2)
  );

  const auto actions = racing::kinematic_model::action::create_actions(5, 15);
  const auto detector = racing::collision_detector::precalculate(120, vehicle, cell_size);
  
  const racing::trajectory_error_calculator error_calculator(
    30.0, // position weight
    20.0, // heading weight
    10.0, // velocity weight
    1.0,
    5.0, // distance to obstacle weight
    vehicle.radius() * 5
  );

  const double integration_step_s = 1.0 / 20.0;
  const double prediction_horizon_s = 0.5;

  racing::kinematic_model::state_transition model(
    std::make_unique<math::euler_method>(integration_step_s), vehicle);

  const int lookahead = int(ceil(prediction_horizon_s / integration_step_s));

  racing::dwa strategy(
    lookahead,
    actions,
    model,
    *detector,
    error_calculator
  );

  Follower follower(strategy);
  
  ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>(map_topic, 1, &Follower::map_observed, &follower);
  ros::Subscriber odometry_sub = node.subscribe<nav_msgs::Odometry>(odometry_topic, 1, &Follower::state_observed, &follower);
  ros::Subscriber trajectory_sub = node.subscribe<racer::TrajectoryMsg>(trajectory_topic, 1, &Follower::trajectory_observed, &follower);
  ros::Subscriber waypoints_sub = node.subscribe<racer::WaypointsMsg>(waypoints_topic, 1, &Follower::waypoints_observed, &follower);

  ros::Publisher command_pub = node.advertise<geometry_msgs::Twist>(driving_topic, 1);
  ros::Publisher visualization_pub = node.advertise<nav_msgs::Path>(visualization_topic, 1, true);

  ros::Rate rate(8); // Hz

  while (ros::ok()) {
    if (follower.is_initialized()) {
      auto action = follower.select_driving_command();

      if (!action) {
        action = std::make_unique<racing::kinematic_model::action>(-1, 0); // stop!
      }

      geometry_msgs::Twist msg;
      msg.linear.x = action->throttle;
      msg.angular.z = action->target_steering_angle;

      command_pub.publish(msg);

      if (visualization_pub.getNumSubscribers() > 0) {
        nav_msgs::Path vis_msg;
        vis_msg.header.frame_id = follower.frame_id;
        vis_msg.header.stamp = ros::Time::now();

        auto state = std::make_unique<racing::kinematic_model::state>(follower.last_known_state());
        for (int i = 0; i < lookahead; ++i) {
          geometry_msgs::PoseStamped segment;

          state = std::move(model.predict(*state, *action));

          segment.header.frame_id = follower.frame_id;
          segment.header.stamp = ros::Time::now();
          segment.pose.position.x = state->position.x;
          segment.pose.position.y = state->position.y;
          segment.pose.position.z = 0;
          segment.pose.orientation = tf::createQuaternionMsgFromYaw(state->position.heading_angle);

          vis_msg.poses.push_back(segment);
        }

        visualization_pub.publish(vis_msg);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
