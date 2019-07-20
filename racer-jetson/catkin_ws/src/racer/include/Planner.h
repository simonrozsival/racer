#ifndef PLANNER_H_
#define PLANNER_H_

#include <list>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "racer/TrajectoryMsg.h"
#include "racer/WaypointsMsg.h"

#include "racing-cpp/astar/hybrid_astar.h"
#include "racing-cpp/racing/kinematic_bicycle_model.h"
#include "racing-cpp/racing/occupancy_grid_collisions.h"

using namespace astar::hybrid_astar;

class Planner {
  public:
    Planner(
      const racing::vehicle_properties& model,
      const std::list<racing::kinematic_model::action>& available_actions,
      const astar::discretization<astar::hybrid_astar::discrete_state>& discretization)
      : model_(model),
      available_actions_(available_actions),
      discretization_(discretization),
      time_step_s_(1.0 / 25.0),
      maximum_number_of_expanded_nodes_(10000),
      collision_detector_(nullptr)
    {
    }

    bool is_initialized() const;
    void initialize(const nav_msgs::OccupancyGrid& map);

    racer::TrajectoryMsg plan(
      const nav_msgs::OccupancyGrid& map,
      const nav_msgs::Odometry& state,
      const racer::WaypointsMsg& waypoints) const;

  private:
    const racing::vehicle_properties model_;
    const double time_step_s_;
    const int maximum_number_of_expanded_nodes_;
    const std::list<racing::kinematic_model::action> available_actions_;
    const astar::discretization<astar::hybrid_astar::discrete_state>& discretization_;

    std::shared_ptr<racing::collision_detector> collision_detector_;
};

#endif