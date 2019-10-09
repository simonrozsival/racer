#ifndef vehicle_configuration_H_
#define vehicle_configuration_H_

#include "racer/math/base_integrator.h"
#include "racer/math/primitives.h"

namespace racer
{

struct vehicle_configuration
{
private:
    racer::math::point location_;
    double heading_angle_;
    bool is_valid_;

public:
    vehicle_configuration(double x, double y, double heading_angle)
        : location_{x, y}, heading_angle_{heading_angle}, is_valid_{true}
    {
    }

    vehicle_configuration(racer::math::point location, double heading_angle)
        : location_{location}, heading_angle_{heading_angle}, is_valid_{true}
    {
    }

    vehicle_configuration()
        : location_{0, 0}, heading_angle_{0}, is_valid_{false}
    {
    }

    vehicle_configuration(const vehicle_configuration &copy_from) = default;
    vehicle_configuration &operator=(const vehicle_configuration &copy_from) = default;

    vehicle_configuration(vehicle_configuration &&pos) = default;
    vehicle_configuration &operator=(vehicle_configuration &&pos)
    {
        pos.is_valid_ = false;
        location_ = std::move(pos.location_);
        heading_angle_ = std::move(pos.heading_angle_);
        is_valid_ = true;
        return *this;
    }

    bool operator==(const vehicle_configuration &other) const
    {
        return location_ == other.location_ && heading_angle_ == other.heading_angle_;
    }

    bool is_valid() const
    {
        return is_valid_;
    }

    vehicle_configuration operator+(const vehicle_configuration &other) const
    {
        return vehicle_configuration(location_ + other.location_, heading_angle_ + other.heading_angle_);
    }

    const racer::math::vector &location() const { return location_; }
    const double &heading_angle() const { return heading_angle_; }

    vehicle_configuration integrate(const std::unique_ptr<racer::math::base_integrator> &integrator) const
    {
        return vehicle_configuration(
            {integrator->integrate(location_.x()), integrator->integrate(location_.y())},
            integrator->integrate(heading_angle_));
    }
};

} // namespace racer

#endif