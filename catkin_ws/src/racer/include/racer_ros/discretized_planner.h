#pragma once

#include <optional>
#include <vector>
#include <ros/ros.h>

#include "nav_msgs/OccupancyGrid.h"

#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"
#include "racer_msgs/State.h"

#include "racer/astar/astar.h"
#include "racer/astar/discretized/search_problem.h"
#include "racer/vehicle/action.h"
#include "racer/vehicle/base_model.h"
#include "racer/track/circuit.h"
#include "racer/track/collision_detection.h"

#include "racer_ros/base_planner.h"
#include "racer_ros/utils.h"

using State = racer::vehicle::kinematic::state;

namespace racer_ros {

template <typename DiscreteState>
class discretized_planner : public base_planner {
public:
  discretized_planner(
    const std::shared_ptr<racer::vehicle::base_vehicle_model<State>> model,
    const std::vector<racer::vehicle::action> available_actions,
    const double time_step_s,
    const double safety_margin,
    ros::Publisher debug_map_pub)
      : model_{model},
        available_actions_{available_actions}
        time_step_s_{time_step_s},
        safety_margin_{safety_margin},
        debug_map_pub_{debug_map_pub} {}

  std::optional<racer_msgs::Trajectory>
  plan() const override {

    if (!discretization_) {
      return {};
    }

    auto problem = std::make_unique<
        racer::astar::discretized::search_problem<DiscreteState, State>>(
        last_known_state_, time_step_s_, available_actions_, discretization_, model_,
        circuit, collision_detector);

    std::atomic<bool> terminate = false;
    const auto result = racer::astar::search<DiscreteState, State>(
        std::move(problem), terminate);

    if (!result.was_successful()) {
      return {};
    }

    return racer_ros::trajectory_to_msg(result.found_trajectory, next_waypoint);
  }

  void load_initial_map() override {
    occupancy_grid_ = racer_ros::load_map(node);
    collision_detector_ = std::make_shared<racer::track::collision_detection>(
        occupancy_grid, model->chassis, 72, safety_margin);
  }

  void map_update(const nav_msgs::OccupancyGrid::ConstPtr &msg) override {
    occupancy_grid_ = racer_ros::msg_to_grid(*msg);
    collision_detector_ = std::make_shared<racer::track::collision_detection>(
        occupancy_grid_, model_->chassis, 72, safety_margin_);

    debug_map_pub_.publish(
        racer_ros::grid_to_msg(*collision_detector_->inflated_grid()));
  }

  void state_update(const racer_msgs::State::ConstPtr &state) override {
    last_known_state_ = racer_ros::msg_to_state(state);
  }

  void waypoints_update(const racer_msgs::Waypoints::ConstPtr &waypoints) override {
    std::lock_guard<std::mutex> guard(mutex);

    // have we already successfully processed these next waypoints?
    if (next_waypoint == waypoints->next_waypoint && planner) {
      return;
    }

    next_waypoints_.clear();
    next_waypoint = waypoints->next_waypoint;

    for (const auto &wp : waypoints->waypoints) {
      next_waypoints_.emplace_back(wp.position.x, wp.position.y);
    }

    const auto waypoint_radius_ = waypoints->waypoints[0].radius;
    circuit_ = std::make_shared<racer::track::circuit>(next_waypoints_, waypoint_radius_,
                                              occupancy_grid_);

    discretization_ = create_discretization();
    if (!discretization_) {
      ROS_ERROR("space exploration failed, goal is inaccessible.");
      return;
    }
  }

protected:
  virtual std::shared_ptr<racer::astar::discretized::discretization<DiscreteState, State>> create_discretization() const = 0;

  const std::shared_ptr<racer::vehicle::model<State>> model_;
  
  State last_known_state_;
  std::shared_ptr<racer::track::occupancy_grid> occupancy_grid_;
  std::shared_ptr<racer::track::circuit> circuit_;
  std::shared_ptr<racer::track::collision_detection> collision_detector_;

  int next_waypoint_;
  std::vector<racer::math::point> next_waypoints_;

private:
  const std::vector<racer::vehicle::action> available_actions_;
  const double time_step_s_, safety_margin_;

  std::mutex mutex;
  std::shared_ptr<racer::astar::discretized::discretization<DiscreteState, State>> discretization_;

  ros::Publisher debug_map_pub_;
};

} // namespace racer_ros
