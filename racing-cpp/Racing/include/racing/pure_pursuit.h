#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_

#include "./kinematic_bicycle_model.h"

namespace racing {

  class pure_pursuit {
  public:
    pure_pursuit(
        const vehicle_properties& vehicle,
        double min_lookahead,
        double lookahead_coef)
      : vehicle_(vehicle), min_lookahead_(min_lookahead), lookahead_coef_(lookahead_coef)
    {
    }

    double select_steering_angle(const kinematic_model::state& current_state, int passed_waypoints, const kinematic_model::trajectory& trajectory) const {
      auto sub_trajectory = trajectory.find_reference_subtrajectory(current_state, passed_waypoints);

      if (sub_trajectory.steps.size() == 0) {
        return 0.0;
      }

      const double lookahead = std::min(min_lookahead_, current_state.speed * lookahead_coef_);
      const double lookahead_sq = lookahead * lookahead;

      auto reference_step = sub_trajectory.steps.begin();
      while (reference_step != sub_trajectory.steps.end() && (reference_step.step.position.location() - current_state.position.location()).length_sq() < lookahead_sq)
      {
        ++reference_step;
      }

      auto reference = reference_step == sub_trajectory.steps.end()
        ? sub_trajectory.steps.back().position
        : reference_step->step.position;

      auto diff = reference.location() - current_state.position.location();
      double alpha = atan2(diff.y, diff.x) - current_state.position.heading_angle;

      return atan(2 * vehicle_.wheelbase * sin(alpha) / lookahead);
    }

  private:
    const vehicle_properties& vehicle_;
    double min_lookahead_;
    double lookahead_coef_;
  };

}

#endif