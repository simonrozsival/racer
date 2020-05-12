#pragma once

#include <vector>

#include "racer/astar/discretized/base_discretization.h"
#include "racer/sehs/nearest_neighbor.h"
#include "racer/sehs/space_exploration.h"
#include "racer/track/occupancy_grid.h"
#include "racer/vehicle/kinematic/state.h"

#include "racer/astar/sehs/discrete_state.h"

namespace racer::astar::sehs {

struct discretization : public racer::astar::discretized::base_discretization<
                            discrete_state, racer::vehicle::kinematic::state> {
private:
  const double motor_rpm_;
  const racer::math::angle heading_;
  const racer::sehs::nearest_neighbor nn_;

public:
  discretization(const std::vector<racer::math::circle> &circles,
                 racer::math::angle heading, double motor_rpm)
      : motor_rpm_{motor_rpm}, heading_{heading}, nn_{centers_of(circles)} {}

  discretization(const discretization &other) = delete;
  discretization(discretization &&other) = delete;

  discrete_state
  operator()(const racer::vehicle::kinematic::state &state) override {
    return {
        (int)nn_.find_nearest_neighbor(state.position()),
        (int)floor(racer::math::angle(state.cfg().heading_angle()) / heading_),
        (int)floor(state.motor_rpm() / motor_rpm_)};
  }

  std::string description() const override {
    std::stringstream s;
    s << "c-" << nn_.size() << "-rpm-" << motor_rpm_ << "-theta-"
      << heading_.to_degrees();
    return s.str();
  }

  static std::unique_ptr<discretization>
  from(racer::vehicle::configuration start,
       std::shared_ptr<racer::track::occupancy_grid> occupancy_grid,
       std::vector<racer::math::point> next_waypoints, double vehicle_radius,
       double max_rpm) {
    racer::sehs::space_exploration exploration{vehicle_radius,
                                               5.0 * vehicle_radius, 16};
    const auto path_of_circles =
        exploration.explore_grid(occupancy_grid, start, next_waypoints);
    if (path_of_circles.empty()) {
      return nullptr;
    }

    double different_angles = 18.0;
    double speed_levels = 20.0;
    return std::make_unique<racer::astar::sehs::discretization>(
        path_of_circles, 2 * M_PI / different_angles, max_rpm / speed_levels);
  }

private:
  static std::vector<racer::math::point>
  centers_of(std::vector<racer::math::circle> circles) {
    std::vector<racer::math::point> centers;
    for (const auto &circle : circles) {
      centers.push_back(circle.center());
    }

    return centers;
  }
};

} // namespace racer::astar::sehs
