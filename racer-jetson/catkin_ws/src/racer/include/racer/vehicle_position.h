#ifndef VEHICLE_POSITION_H_
#define VEHICLE_POSITION_H_

#include "racer/math/base_integrator.h"
#include "racer/math/primitives.h"

namespace racer {

    struct vehicle_position {
        const double x, y, heading_angle;

        vehicle_position(double x, double y, double heading_angle)
            : x(x), y(y), heading_angle(heading_angle)
        {
        }

        bool operator==(const vehicle_position& other) const {
            return x == other.x && y == other.y && heading_angle == other.heading_angle;
        }

        vehicle_position operator+(const vehicle_position& other) const {
            return vehicle_position(x + other.x, y + other.y, heading_angle + other.heading_angle);
        }

        racer::math::vector location() const {
            return racer::math::vector(x, y);
        }

        vehicle_position integrate(const std::unique_ptr<racer::math::base_integrator>& integrator) const {
            return vehicle_position(
                integrator->integrate(x),
                integrator->integrate(y),
                integrator->integrate(heading_angle)
            );
        }
    };

}

#endif
