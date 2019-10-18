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
#include "racer/vehicle_model/kinematic_model.h"
#include "racer/occupancy_grid.h"
#include "racer/vehicle_configuration.h"

namespace racer_ros
{

template <typename State, typename DiscreteState>
class Planner
{
public:
  Planner(
      std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model,
      std::unique_ptr<racer::astar::discretization<DiscreteState, State>> discretization,
      const double time_step_s,
      const std::string map_frame_id)
      : model_(model),
        discretization_(std::move(discretization)),
        time_step_s_(time_step_s),
        maximum_number_of_expanded_nodes_(10000),
        map_frame_(map_frame_id)
  {
  }

  std::unique_ptr<racer_msgs::Trajectory> plan(
      std::shared_ptr<racer::occupancy_grid> grid,
      const State &initial_state,
      const std::vector<racer::action> &available_actions,
      const std::shared_ptr<racer::circuit> circuit,
      const int next_waypoint) const;

private:
  const std::shared_ptr<racer::vehicle_model::vehicle_model<State>> model_;
  const double time_step_s_;
  const int maximum_number_of_expanded_nodes_;
  const std::shared_ptr<racer::astar::discretization<DiscreteState, State>> discretization_;
  const std::string map_frame_;
};

} // namespace racer_ros

#endif
