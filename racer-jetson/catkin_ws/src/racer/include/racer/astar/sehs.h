#pragma once

#include <vector>

#include "racer/astar/discretized_search_problem.h"
#include "racer/nearest_neighbor.h"
#include "racer/vehicle_model/kinematic_model.h"

namespace racer::astar::sehs::kinematic
{
struct discrete_state
{
private:
  int circle_, heading_, rpm_;

  friend class std::hash<std::pair<racer::astar::sehs::kinematic::discrete_state, size_t>>;

public:
  discrete_state() : circle_{ 0 }, heading_{ 0 }, rpm_{ 0 }
  {
  }

  discrete_state(int circle, int heading, int rpm) : circle_{ circle }, heading_{ heading }, rpm_{ rpm }
  {
  }

  discrete_state(const discrete_state &other) = default;
  discrete_state &operator=(const discrete_state &other) = default;

  discrete_state(discrete_state &&other) = default;
  discrete_state &operator=(discrete_state &&other) = default;

  bool operator==(const discrete_state &other) const
  {
    return circle_ == other.circle_ && heading_ == other.heading_ && rpm_ == other.rpm_;
  }

  bool operator!=(const discrete_state &other) const
  {
    return !(*this == other);
  }
};

struct discretization : public racer::astar::discretization<discrete_state, racer::vehicle_model::kinematic::state>
{
private:
  const double motor_rpm_;
  const racer::math::angle heading_;
  // const racer::nearest_neighbor::cached_linear_search nn_;
  const racer::nearest_neighbor::linear_search nn_;
  // const racer::nearest_neighbor::kd_tree::tree nn_;

public:
  discretization(const std::vector<racer::math::circle> &circles, racer::math::angle heading, double motor_rpm)
    : motor_rpm_{ motor_rpm }, heading_{ heading }, nn_{ centers_of(circles) }
  {
  }

  discretization(const discretization &other) = delete;
  discretization(discretization &&other) = delete;

  discrete_state operator()(const racer::vehicle_model::kinematic::state &state) override
  {
    return { (int)nn_.find_nearest_neighbor(state.position()),
             (int)floor(racer::math::angle(state.configuration().heading_angle()) / heading_),
             (int)floor(state.motor_rpm() / motor_rpm_) };
  }

  std::string description() const override
  {
    std::stringstream s;
    s << "c-" << nn_.size() << "-rpm-" << motor_rpm_ << "-theta-" << heading_.to_degrees();
    return s.str();
  }

  static std::unique_ptr<discretization> from(racer::vehicle_configuration start,
                                              std::shared_ptr<racer::occupancy_grid> occupancy_grid,
                                              std::vector<racer::math::point> next_waypoints, double vehicle_radius,
                                              double max_rpm)
  {
    racer::sehs::space_exploration exploration{ vehicle_radius, 5.0 * vehicle_radius, 8 };
    const auto path_of_circles = exploration.explore_grid(occupancy_grid, start, next_waypoints);
    if (path_of_circles.empty())
    {
      return nullptr;
    }

    double different_angles = 12.0;
    double speed_levels = 10.0;
    return std::make_unique<racer::astar::sehs::kinematic::discretization>(path_of_circles, 2 * M_PI / different_angles,
                                                                           max_rpm / speed_levels);
  }

private:
  static std::vector<racer::math::point> centers_of(std::vector<racer::math::circle> circles)
  {
    std::vector<racer::math::point> centers;
    for (const auto &circle : circles)
    {
      centers.push_back(circle.center());
    }

    return centers;
  }
};

}  // namespace racer::astar::sehs::kinematic

namespace std
{
template <>
struct hash<std::pair<racer::astar::sehs::kinematic::discrete_state, size_t>>
{
  size_t operator()(const std::pair<racer::astar::sehs::kinematic::discrete_state, size_t> &obj) const
  {
    size_t seed = 0;

    racer::math::hash_combine(seed, obj.first.circle_);
    racer::math::hash_combine(seed, obj.first.heading_);
    racer::math::hash_combine(seed, obj.first.rpm_);
    racer::math::hash_combine(seed, obj.second);

    return seed;
  }
};

}  // namespace std
