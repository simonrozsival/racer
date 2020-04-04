#pragma once

#include <cstdlib>

namespace racer::following_strategies
{
class pid
{
public:
  pid(double kp, double ki, double kd, double error_tolerance)
    : kp_(kp), ki_(ki), kd_(kd), last_error_(0), accumulated_error_(0), error_tolerance_(error_tolerance)
  {
  }

  pid(const pid& other) = default;
  pid& operator=(const pid& other) = default;

  pid(pid&& other) = default;
  pid& operator=(pid&& other) = default;

  double predict_next(const double current, const double target)
  {
    double error = target - current;
    double error_derivative = last_error_ - error;

    last_error_ = error;

    if (std::abs(error) < error_tolerance_)
    {
      accumulated_error_ = 0;
    }
    else
    {
      accumulated_error_ += error;
    }

    return kp_ * error + ki_ * accumulated_error_ + kd_ * error_derivative;
  }

private:
  double kp_, ki_, kd_;
  double last_error_, accumulated_error_, error_tolerance_;
};

}  // namespace racer::following_strategies
