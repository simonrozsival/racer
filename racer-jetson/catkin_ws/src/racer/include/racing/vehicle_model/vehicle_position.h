#ifndef VEHICLE_POSITION_H_
#define VEHICLE_POSITION_H_

#include "math/base_integrator.h"
#include "math/primitives.h"

namespace racing {

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

        math::vector location() const {
            return math::vector(x, y);
        }

        vehicle_position integrate(const std::unique_ptr<math::base_integrator>& integrator) const {
            return vehicle_position(
                integrator->integrate(x),
                integrator->integrate(y),
                integrator->integrate(heading_angle)
            );
        }
    };

}

#endif
