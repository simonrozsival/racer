#ifndef PLANNER_H_
#define PLANNER_H_

#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"

#include "racer/math.h"

#include "racer/astar/sehs.h"

#include "racer/action.h"
#include "racer/trajectory.h"
#include "racer/vehicle_model/vehicle.h"
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/occupancy_grid.h"

using namespace racer;
using namespace racer::astar::sehs;
using namespace racer::vehicle_model;

namespace racer_ros
{

class Planner
{
public:
  Planner(
      vehicle &&vehicle,
      std::unique_ptr<racer::astar::discretization<discrete_state>> discretization,
      const double time_step_s,
      const std::string map_frame_id)
      : vehicle_(vehicle),
        discretization_(std::move(discretization)),
        time_step_s_(time_step_s),
        maximum_number_of_expanded_nodes_(10000),
        map_frame_(map_frame_id)
  {
  }

  std::unique_ptr<racer_msgs::Trajectory> plan(
      const occupancy_grid &grid,
      const kinematic::state &initial_state,
      const std::vector<action> &available_actions,
      const std::vector<racer::math::point> &waypoints,
      const int next_waypoint,
      const double waypoint_radius) const;

private:
  const vehicle vehicle_;
  const double time_step_s_;
  const int maximum_number_of_expanded_nodes_;
  const std::shared_ptr<racer::astar::discretization<discrete_state>> discretization_;
  const std::string map_frame_;
};

} // namespace racer_ros

#endif
