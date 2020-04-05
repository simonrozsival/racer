#pragma once

#include <iostream>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/vehicle_model/base_model.h"
#include "racer/vehicle_model/vehicle_chassis.h"

namespace racer::following_strategies {

template <typename State> class target_error_calculator {
public:
  target_error_calculator()
      : position_error_weight_{0}, heading_error_weight_{0},
        motor_rpm_error_weight_{0}, obstacle_proximity_error_weight_{0} {}

  target_error_calculator(double position_error_weight,
                          double heading_error_weight,
                          double motor_rpm_error_weight,
                          double obstacle_proximity_error_weight,
                          racer::vehicle_model::rpm max_rpm)
      : position_error_weight_(position_error_weight),
        heading_error_weight_(heading_error_weight),
        motor_rpm_error_weight_(motor_rpm_error_weight),
        obstacle_proximity_error_weight_(obstacle_proximity_error_weight),
        max_rpm_(max_rpm) {}

  target_error_calculator(target_error_calculator<State> &&other) = default;
  target_error_calculator<State> &
  operator=(target_error_calculator<State> &&other) = default;

  target_error_calculator(const target_error_calculator<State> &other) =
      default;
  target_error_calculator<State> &
  operator=(const target_error_calculator<State> &other) = default;

  double
  calculate_error(const std::vector<State> &prediction,
                  const racer::trajectory<State> &reference,
                  const std::shared_ptr<racer::occupancy_grid> map) const {
    double total_error = 0;
    auto it_a{prediction.begin()};
    auto it_b{reference.steps().begin()};

    std::size_t steps = 0;

    for (; it_a != prediction.end() && it_b != reference.steps().end();
         ++it_a, ++it_b) {
      const double err = calculate_error(*it_a, it_b->state(), map);
      const double discount =
          (prediction.size() - ++steps) / double(prediction.size());
      total_error += discount * err;
    }

    return total_error / double(prediction.size());
  }

  double
  calculate_error(const State a, const State b,
                  const std::shared_ptr<racer::occupancy_grid> map) const {
    return position_error_weight_ * position_error(a, b) +
           heading_error_weight_ * heading_error(a, b) +
           motor_rpm_error_weight_ * motor_rpm_error(a, b) +
           obstacle_proximity_error_weight_ * obstacle_proximity_error(a, map);
  }

private:
  double position_error_weight_, heading_error_weight_, motor_rpm_error_weight_;
  double obstacle_proximity_error_weight_, max_rpm_;

  double position_error(const State &a, const State &reference) const {
    return a.position().distance_sq(reference.position());
  }

  double heading_error(const State &a, const State &reference) const {
    const auto heading_a = a.configuration().heading_angle().to_normal_angle();
    const auto heading_b =
        reference.configuration().heading_angle().to_normal_angle();

    return heading_a.distance_to(heading_b) / M_PI;
  }

  double motor_rpm_error(const State &a, const State &reference) const {
    return std::abs(reference.motor_rpm() - a.motor_rpm()) / max_rpm_;
  }

  double obstacle_proximity_error(
      const State &a, const std::shared_ptr<racer::occupancy_grid> grid) const {
    const double ideal_distance = 2.0;
    const double dist =
        grid->distance_to_closest_obstacle(a.position(), ideal_distance);
    return (ideal_distance - dist) / ideal_distance;
  }
};

} // namespace racer::following_strategies