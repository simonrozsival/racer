#ifndef ACTION_H_
#define ACTION_H_

#include <iostream>

namespace racer
{

struct action
{
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

    action(const action &action) = default;
    action &operator=(const action &action) = default;

    action(action &&action) = default;
    action &operator=(action &&action) = default;

    const double &throttle() const { return throttle_; }
    const double &target_steering_angle() const { return target_steering_angle_; }
    const bool is_valid() const { return is_valid_; }

    static const std::list<action> create_actions(const int throttle_levels, const int steering_levels)
    {
        std::list<action> actions;

        const double steering_step = 2.0 / double(steering_levels - 1);
        const double throttle_step = 1.0 / std::max(1.0, double(throttle_levels - 1));

        for (double throttle = 1; throttle > 0; throttle -= throttle_step)
        {
            for (double steering = -1; steering <= 1; steering += steering_step)
            {
                actions.emplace_back(throttle, steering);
            }
        }

        return actions;
    }

    static std::list<action> create_actions_including_reverse(const int throttle_levels, const int steering_levels)
    {
        std::list<action> actions;

        const double steering_step = 2.0 / double(steering_levels - 1);
        const double throttle_step = 2.0 / double(throttle_levels - 1);

        for (double throttle = 1; throttle >= -1; throttle -= throttle_step)
        {
            for (double steering = -1; steering <= 1; steering += steering_step)
            {
                actions.emplace_back(throttle, steering);
            }
        }

        return actions;
    }
};

} // namespace racer

#endif
