#pragma once

#include <optional>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"

#include "racer_msgs/Trajectory.h"
#include "racer_msgs/Waypoints.h"
#include "racer_msgs/State.h"

using State = racer::vehicle::kinematic::state;

namespace racer_ros {

class base_planner {
public:
  virtual std::optional<racer_msgs::Trajectory> plan() const = 0;
  virtual void map_update(const nav_msgs::OccupancyGrid::ConstPtr &msg) = 0;
  virtual void state_update(const racer_msgs::State::ConstPtr &state) = 0;
  virtual void waypoints_update(const racer_msgs::Waypoints::ConstPtr &waypoints) = 0;
};

} // namespace racer_ros
