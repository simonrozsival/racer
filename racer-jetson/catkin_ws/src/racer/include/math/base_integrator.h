#ifndef BASE_INTEGRATOR_H_
#define BASE_INTEGRATOR_H_

namespace math {

    class base_integrator {
    public:
        base_integrator(double integration_time_ms)
            : integration_time_ms_(integration_time_ms)
        {
        }

        virtual double integrate(double derivative) const = 0;
    protected:
        const double integration_time_ms_;
    };

}

#endif