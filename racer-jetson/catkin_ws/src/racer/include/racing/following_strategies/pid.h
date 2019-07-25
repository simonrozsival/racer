#ifndef PID_H_
#define PID_H_

#include <cstdlib>

namespace racing {

    class pid {
    public:
        pid(double kp, double ki, double kd, double error_tolerance)
            : kp_(kp), ki_(ki), kd_(kd), last_error_(0), accumulated_error_(0), error_tolerance_(error_tolerance)
        {
        }

        double predict_next(const double current, const double target) {
            double error = target - current;
            double error_derivative = last_error_ - error;
            
            last_error_ = error;
            
            if (std::abs(error) < error_tolerance_) {
                accumulated_error_ = 0;
            } else {
                accumulated_error_ += error;
            }

            return kp_ * error + ki_ * accumulated_error_ + kd_ * error_derivative;
        }

        void reconfigure(double kp, double ki, double kd, double error_tolerance) {
            kp_ = kp;
            ki_ = ki;
            kd_ = kd;
            error_tolerance_ = error_tolerance;
        }

        double reset() {
            last_error_ = 0;
            accumulated_error_ = 0;
        }
    private:
        double kp_, ki_, kd_;
        double last_error_, accumulated_error_, error_tolerance_;
    };

}

#endif