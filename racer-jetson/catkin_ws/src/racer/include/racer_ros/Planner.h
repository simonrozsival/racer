#ifndef PLANNER_H_
#define PLANNER_H_

#include <list>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racer/math/primitives.h"  

#include "racer/astar/sehs.h"

#include "racer/vehicle_model/vehicle.h"
#include "racer/vehicle_model/kinematic_bicycle_model.h"
#include "racer/occupancy_grid.h"

using namespace racer;
using namespace racer::astar::sehs;

namespace racer_ros {

  class Planner {
    public:
      Planner(
        const racer::vehicle_model::vehicle& model,
        const racer::astar::discretization<discrete_state>& discretization,
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
        const racer::occupancy_grid& grid,
        const racer::vehicle_model::kinematic_bicycle_model::state& initial_state,
        const std::list<racer::vehicle_model::kinematic_bicycle_model::action>& available_actions,
        const std::vector<racer::math::point>& waypoints,
        const int next_waypoint,
        const double waypoint_radius) const;

    private:
      const racer::vehicle_model::vehicle model_;
      const double time_step_s_;
      const int maximum_number_of_expanded_nodes_;
      const racer::astar::discretization<discrete_state>& discretization_;
      const std::string map_frame_;
  };

}

#endif
