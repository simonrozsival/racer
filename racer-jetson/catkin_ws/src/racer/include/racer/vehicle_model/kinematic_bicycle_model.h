#ifndef KINEMATIC_BICYCLE_MODEL_H_
#define KINEMATIC_BICYCLE_MODEL_H_

#include <iostream>
#include <algorithm>
#include <cmath>

#include "racer/math/base_integrator.h"
#include "racer/vehicle_position.h"

#include "./vehicle.h"
#include "./base_vehicle_model.h"

namespace racer::vehicle_model::kinematic_bicycle_model {
        
    struct action {
    private:
        double throttle_, target_steering_angle_;
        bool is_valid_;

    public:
        action() : throttle_{0}, target_steering_angle_{0}, is_valid_{false}
        {
        }

        action(double throttle, double target_steering_angle)
            : throttle_(throttle), target_steering_angle_(target_steering_angle), is_valid_{true}
        {
        }

        action(const action& action) = default;
        action& operator=(const action& action) = default;

        action(action&& action) = default;
        action& operator=(action&& action) = default;

        constexpr const double& throttle() const { return throttle_; }
        constexpr const double& target_steering_angle() const { return target_steering_angle_; }
        constexpr const bool is_valid() const { return is_valid_; }

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

        static std::list<action> create_actions_including_reverse(const int throttle_levels, const int steering_levels) {
            std::list<action> actions;

            const double steering_step = 2.0 / double(steering_levels - 1);
            const double throttle_step = 2.0 / double(throttle_levels - 1);

            for (double throttle = 1; throttle >= -1; throttle -= throttle_step) {
                for (double steering = -1; steering <= 1; steering += steering_step) {
                    actions.emplace_back(throttle, steering);
                }
            }

            return actions;
        }
    };

    struct state {
    private:
        vehicle_position position_;
        double speed_, steering_angle_;

    public:
        state()
            : position_{},
              speed_{0},
              steering_angle_{0}
        {
        }

        state(vehicle_position position, double speed, double steering_angle)
            : position_(position), speed_(speed), steering_angle_(steering_angle)
        {
        }
    
        state(const state& s) = default;
        state& operator=(const state& s) = default;

        state(state&& s) = default;
        state& operator=(state&& s) = default;

        constexpr const vehicle_position& position() const noexcept { return position_; }
        constexpr const double& speed() const noexcept { return speed_; }
        constexpr const double& steering_angle() const noexcept { return steering_angle_; }

        bool operator ==(const state& other) const {
            return position_ == other.position_ && speed_ == other.speed_ && steering_angle_ == other.steering_angle_;
        }

        bool operator !=(const state& other) const {
            return !(*this == other);
        }

        double distance_to(const state& other) const {
            return (position_.location() - other.position_.location()).length();
        }

        bool is_valid() const noexcept { return position_.is_valid(); }
    };

    struct trajectory_step {
    private:
        state step_;
        std::size_t passed_waypoints_;

    public:
        trajectory_step()
            : step_{}, passed_waypoints_{0}
        {
        }

        trajectory_step(const state& step, const std::size_t passed_waypoints)
            : step_(step), passed_waypoints_(passed_waypoints)
        {
        }

        trajectory_step(const trajectory_step& step) = default;
        trajectory_step& operator=(const trajectory_step& step) = default;

        trajectory_step(trajectory_step&& step) = default;
        trajectory_step& operator=(trajectory_step&& step) = default;

        constexpr const state& step() const noexcept { return step_; }
        constexpr const std::size_t& passed_waypoints() const noexcept { return passed_waypoints_; }
    };

    struct trajectory {
    private:
        std::list<trajectory_step> steps_;

    public:
        trajectory() : steps_{}
        {
        }

        trajectory(const std::list<trajectory_step> steps)
            : steps_(steps)
        {
        }
        
        trajectory(const trajectory& traj) = default;
        trajectory& operator=(const trajectory& traj) = default;

        trajectory(trajectory&& traj) = default;
        trajectory& operator=(trajectory&& traj) = default;

        trajectory find_reference_subtrajectory(const state& current_state, std::size_t passed_waypoints) const {
            const auto reference_state = find_reference_state(current_state, passed_waypoints);
            std::list<trajectory_step> sublist{ reference_state, steps_.end() };
            return { sublist };
        }

        constexpr const std::list<trajectory_step>& steps() const noexcept { return steps_; }

        bool empty() const noexcept { return steps_.empty(); }
        bool is_valid() const noexcept { return !steps_.empty(); }

    private:
        std::list<trajectory_step>::const_iterator find_reference_state(const state& state, const std::size_t passed_waypoints) const noexcept {
            if (steps_.empty()) {
                return steps_.end();
            }

            auto best_so_far = steps_.begin();
            double distance = HUGE_VAL;
            int i = 0;

            for (auto it = steps_.begin(); it != steps_.end(); ++it) {
                if (it->passed_waypoints() < passed_waypoints) {
                    continue;
                } else if (it->passed_waypoints() > passed_waypoints + 1) { // do not skip a waypoint
                    break;
                }
                ++i;
                double next_distance = state.distance_to(it->step());
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
        const std::unique_ptr<racer::math::base_integrator> integrator_;
        const vehicle vehicle_;

    public:
        model(std::unique_ptr<racer::math::base_integrator> integrator, const vehicle& vehicle)
            : integrator_(std::move(integrator)), vehicle_(vehicle)
        {
        }

    public:
        state predict(const state& current, const action& input) const override {
            double speed = calculate_speed(current, input);
            double steering_angle = calculate_steering_angle(current, input);
            racer::math::angle slip_angle = atan((vehicle_.distance_of_center_of_gravity_to_rear_axle / vehicle_.wheelbase) * tan(racer::math::angle(steering_angle)));

            vehicle_position position_derivative {
                speed * cos(current.position().heading_angle() + slip_angle),
                speed * sin(current.position().heading_angle() + slip_angle),
                speed * cos(slip_angle) * tan(racer::math::angle(steering_angle)) / vehicle_.wheelbase
            };

            return { current.position() + position_derivative.integrate(integrator_), speed, steering_angle };
        }

    private:
        double calculate_speed(const state& current, const action& input) const {
            double target_speed = input.throttle() * vehicle_.max_speed;
            double acceleration = sign(target_speed - current.speed()) * vehicle_.max_acceleration;
            double speed_delta = integrator_->integrate(acceleration);

            return abs(current.speed() - target_speed) < abs(speed_delta)
                ? target_speed // don't overshoot
                : std::min(vehicle_.max_speed, std::max(vehicle_.max_reversing_speed, current.speed() + speed_delta));
        }

        double calculate_steering_angle(const state& current, const action& input) const {
            double target_steering_angle = input.target_steering_angle() * vehicle_.max_steering_angle;
            double speed = sign(target_steering_angle - current.steering_angle()) * vehicle_.max_steering_speed;
            double delta = integrator_->integrate(speed);

            return abs(target_steering_angle - current.steering_angle()) < abs(speed)
                ? target_steering_angle
                : std::min(vehicle_.max_steering_angle, std::max(-vehicle_.max_steering_angle, current.steering_angle() + delta));
        }

        double sign(double x) const {
            if (x > 0.0) return 1;
            if (x < 0.0) return -1;
            return 0;
        }
    };

}

#endif
