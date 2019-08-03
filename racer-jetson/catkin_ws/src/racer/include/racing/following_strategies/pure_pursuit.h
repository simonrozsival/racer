#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include "racing/vehicle_model/vehicle.h"
#include "racing/vehicle_model/kinematic_bicycle_model.h"

#include "racing/math/primitives.h"

namespace racing {

  class pure_pursuit {
  public:
    pure_pursuit(
        const vehicle& vehicle,
        double min_lookahead,
        double lookahead_coef)
      : vehicle_(vehicle), min_lookahead_(min_lookahead), lookahead_coef_(lookahead_coef)
    {
    }

    double select_steering_angle(const kinematic_model::state& current_state, int passed_waypoints, const kinematic_model::trajectory& trajectory) const {
      const double lookahead = std::min(min_lookahead_, current_state.speed * lookahead_coef_);
      const auto reference = find_reference_position(
        current_state, passed_waypoints, trajectory, lookahead);

      auto diff = reference.location() - calculate_rear_axle_center(center);
      double alpha = atan2(diff.y, diff.x) - current_state.position.heading_angle;

      return atan(2 * vehicle_.wheelbase * sin(alpha) / lookahead);
    }

    vehicle_position find_reference_position(
      const kinematic_model::state& current_state,
      int passed_waypoints,
      const kinematic_model::trajectory& trajectory,
      const double lookahead) const {
      auto sub_trajectory = trajectory.find_reference_subtrajectory(current_state, passed_waypoints);
      
      if (!sub_trajectory || sub_trajectory->steps.size() == 0) {
        return 0.0;
      }

      auto rear_axle_center = calculate_rear_axle_center(current_state);

      const double lookahead_sq = lookahead * lookahead;
      auto reference_step = sub_trajectory->steps.begin();
      while (reference_step != sub_trajectory->steps.end() && (reference_step->step.position.distance_sq(rear_axle_center) < lookahead_sq) {
        ++reference_step;
      }

      return reference_step == sub_trajectory->steps.end()
        ? sub_trajectory->steps.back().step.position
        : reference_step->step.position;
    }

  private:
    const vehicle& vehicle_;
    double min_lookahead_;
    double lookahead_coef_;

    math::point calculate_rear_axle_center(const kinematic_model::state& current_state) const {
      auto rear_wheel_offset = math::vector(-vehicle_.wheelbase / 2).rotate(current_state.position.heading_angle);
      return current_state.position.location() + rear_wheel_offset;
    }
  };

}

#endif