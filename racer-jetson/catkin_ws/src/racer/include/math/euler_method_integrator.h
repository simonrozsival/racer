#ifndef EULER_METHOD_INTEGRATOR_H_
#define EULER_METHOD_INTEGRATOR_H_

#include <iostream>

#include "base_integrator.h"

namespace math {

    class euler_method : public base_integrator {
    public:
        euler_method(double integration_time_ms)
            : base_integrator(integration_time_ms)
        {
        }

        double integrate(double derivative) const override {
            return derivative * integration_time_ms_;
        }
    };

}

#endif