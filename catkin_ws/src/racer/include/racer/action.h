#pragma once

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

    const double &throttle() const
    {
      return throttle_;
    }
    const double &target_steering_angle() const
    {
      return target_steering_angle_;
    }
    const bool is_valid() const
    {
      return is_valid_;
    }

    static const std::vector<action> create_actions(const std::size_t throttle_levels, const std::size_t steering_levels)
    {
      return create_actions(throttle_levels, steering_levels, 0.0, 1.0, -1.0, 1.0);
    }

    static const std::vector<action> create_actions(const std::size_t throttle_levels, const std::size_t steering_levels,
                                                    const double min_throttle, const double max_throttle, const double max_right, const double max_left)
    {
      std::vector<action> actions;

      if (throttle_levels < 3 || steering_levels < 3)
      {
        throw std::runtime_error("There have to be at least 3 throttle levels and 3 steering levels.");
      }

      const double steering_step = (max_left - max_right) / double(steering_levels - 1);
      const double throttle_step = (max_throttle - min_throttle) / std::max(1.0, double(throttle_levels - 2));

      for (double throttle = max_throttle; throttle >= min_throttle; throttle -= throttle_step)
      {
        for (double steering = max_right; steering <= max_left; steering += steering_step)
        {
          actions.emplace_back(throttle, steering);
        }
      }

      return actions;
    }

    friend std::ostream &operator<<(std::ostream &os, const action &a)
    {
      return os << "(throttle: " << a.throttle_ << ", steering angle: " << a.target_steering_angle_ << ")";
    }
  };

} // namespace racer
