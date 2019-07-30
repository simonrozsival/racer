#ifndef AGENT_H_
#define AGENT_H_

#include <iostream>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racing/following_strategies/following_strategy.h"
#include "racing/collision_detection/occupancy_grid_collision_detector.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"

class Follower {
  public:
    std::string frame_id;

    Follower(std::unique_ptr<racing::following_strategy> strategy)
      : strategy_(std::move(strategy)), next_waypoint_(0), stop_(racing::kinematic_model::action(0, 0))
    {
    }

    bool is_initialized() const;

    void costmap_observed(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void state_observed(const nav_msgs::Odometry::ConstPtr& odometry);
    void trajectory_observed(const racer_msgs::Trajectory::ConstPtr& trajectory);
    void waypoints_observed(const racer_msgs::Waypoints::ConstPtr& waypoints);

    std::unique_ptr<racing::kinematic_model::action> select_driving_command() const;
    std::unique_ptr<racing::kinematic_model::action> stop() const;

    racing::kinematic_model::state last_known_state() const {
      return *last_known_state_;
    }

  private:
    const std::unique_ptr<racing::following_strategy> strategy_;
    const racing::kinematic_model::action stop_;
    int next_waypoint_;

    double last_update_;
    std::unique_ptr<racing::occupancy_grid> costmap_;
    std::unique_ptr<racing::kinematic_model::state> last_known_state_;
    std::unique_ptr<racing::kinematic_model::trajectory> reference_trajectory_;
};

#endif
