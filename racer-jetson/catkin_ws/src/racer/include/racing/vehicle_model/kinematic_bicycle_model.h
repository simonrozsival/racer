#ifndef KINEMATIC_BICYCLE_MODEL_H_
#define KINEMATIC_BICYCLE_MODEL_H_

#include <iostream>
#include <algorithm>

#include "vehicle_position.h"
#include "vehicle.h"
#include "base_vehicle_model.h"
#include "math/base_integrator.h"

namespace racing {
    namespace kinematic_model {
        
        struct action {
            const double throttle, target_steering_angle;

            action(double throttle, double target_steering_angle)
                : throttle(throttle), target_steering_angle(target_steering_angle)
            {
            }

            static const std::list<action> create_actions(const int throttle_levels, const int steering_levels) {
                std::list<action> actions;

                const double steering_step = 2.0 / double(steering_levels - 1);
                const double throttle_step = 1.0 / std::max(1.0, double(throttle_levels - 1));

                for (double throttle = 1; throttle > 0; throttle -= throttle_step) {
                    for (double steering = -1; steering <= 1; steering += steering_step) {
                        actions.emplace_back(throttle, steering);
                    }
                }

                return actions;
            }
        };

        struct state {
            const vehicle_position position;
            const double speed, steering_angle;

            state(vehicle_position position, double speed, double steering_angle)
                : position(position), speed(speed), steering_angle(steering_angle)
            {
            }

            bool operator ==(const state& other) const {
                return position == other.position && speed == other.speed && steering_angle == other.steering_angle;
            }

            bool operator !=(const state& other) const {
                return !(*this == other);
            }

            double distance_to(const state& other) const {
                return (position.location() - other.position.location()).length();
            }
        };

        struct trajectory_step {
            const state step;
            const std::size_t passed_waypoints;

            trajectory_step(const state& step, const std::size_t passed_waypoints)
                : step(step), passed_waypoints(passed_waypoints)
            {
            }
        };

        struct trajectory {
            const std::list<trajectory_step> steps;

            trajectory(const std::list<trajectory_step> steps)
                : steps(steps)
            {
            }

            std::unique_ptr<trajectory> find_reference_subtrajectory(const state& current_state, std::size_t passed_waypoints) const {
                const auto reference_state = find_reference_state(current_state, passed_waypoints);
                std::list<trajectory_step> sublist{ reference_state, steps.end() };
                return std::make_unique<trajectory>(sublist);
            }

            bool empty() const {
                return steps.empty();
            }

        private:
            std::list<trajectory_step>::const_iterator find_reference_state(const state& state, const std::size_t passed_waypoints) const {
                if (steps.empty()) {
                    return steps.end();
                }

                auto best_so_far = steps.begin();
                double distance = 10000000000; // "infinity"
                int i = 0;

                for (auto it = steps.begin(); it != steps.end(); ++it) {
                    if (it->passed_waypoints < passed_waypoints) {
                        continue;
                    } else if (it->passed_waypoints > passed_waypoints + 1) { // do not skip a waypoint
                        break;
                    }
                    ++i;
                    double next_distance = state.distance_to(it->step);
                    if (next_distance < distance) {
                        best_so_far = it;
                        distance = next_distance;
                    }
                }

                return best_so_far;
            }
        };

        class model : base_vehicle_model<state, action> {
        private:
            const std::unique_ptr<math::base_integrator> integrator_;
            const vehicle vehicle_;

        public:
            model(std::unique_ptr<math::base_integrator> integrator, const vehicle& vehicle)
                : integrator_(std::move(integrator)), vehicle_(vehicle)
            {
            }

        public:
            std::unique_ptr<state> predict(const state& current, const action& input) const override {
                double speed = calculate_speed(current, input);
                double steering_angle = calculate_steering_angle(current, input);
                math::angle slip_angle = atan((vehicle_.distance_of_center_of_gravity_to_rear_axle / vehicle_.wheelbase) * tan(math::angle(steering_angle)));

                vehicle_position position_derivative(
                    speed * cos(current.position.heading_angle + slip_angle),
                    speed * sin(current.position.heading_angle + slip_angle),
                    speed * cos(slip_angle) * tan(math::angle(steering_angle)) / vehicle_.wheelbase);

                return std::make_unique<state>(current.position + position_derivative.integrate(integrator_), speed, steering_angle);
            }

        private:
            double calculate_speed(const state& current, const action& input) const {
                double target_speed = input.throttle * vehicle_.max_speed;
                double acceleration = sign(target_speed - current.speed) * vehicle_.max_acceleration;
                double speed_delta = integrator_->integrate(acceleration);

                return abs(current.speed - target_speed) < abs(speed_delta)
                    ? target_speed // don't overshoot
                    : std::min(vehicle_.max_speed, std::max(0.0, current.speed + speed_delta));
            }

            double calculate_steering_angle(const state& current, const action& input) const {
                double target_steering_angle = input.target_steering_angle * vehicle_.max_steering_angle;
                double speed = sign(target_steering_angle - current.steering_angle) * vehicle_.max_steering_speed;
                double delta = integrator_->integrate(speed);

                return abs(target_steering_angle - current.steering_angle) < abs(speed)
                    ? target_steering_angle
                    : std::min(vehicle_.max_steering_angle, std::max(-vehicle_.max_steering_angle, current.steering_angle + delta));
            }

            double sign(double x) const {
                if (x > 0.0) return 1;
                if (x < 0.0) return -1;
                return 0;
            }
        };

    }
}

#endif
