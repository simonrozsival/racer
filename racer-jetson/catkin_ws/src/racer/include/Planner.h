#ifndef PLANNER_H_
#define PLANNER_H_

#include <list>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "math/primitives.h"  

#include "astar/sehs.h"

#include "racing/vehicle_model/vehicle.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"
#include "racing/collision_detection/occupancy_grid.h"

using namespace astar::sehs;

class Planner {
  public:
    Planner(
      const racing::vehicle& model,
      const astar::discretization<discrete_state>& discretization,
      const double time_step_s,
      const std::string map_frame_id)
      : model_(model),
      discretization_(discretization),
      time_step_s_(time_step_s),
      maximum_number_of_expanded_nodes_(10000),
      map_frame_(map_frame_id)
    {
    }

    bool is_initialized() const;

    std::unique_ptr<racer_msgs::Trajectory> plan(
      const std::shared_ptr<racing::occupancy_grid> grid,
      const std::shared_ptr<racing::kinematic_model::state> initial_state,
      const std::list<racing::kinematic_model::action>& available_actions,
      const std::shared_ptr<std::vector<math::point>> waypoints,
      const int next_waypoint,
      const double waypoint_radius) const;

  private:
    const racing::vehicle model_;
    const double time_step_s_;
    const int maximum_number_of_expanded_nodes_;
    const astar::discretization<discrete_state>& discretization_;

    std::string map_frame_;
};

#endif
