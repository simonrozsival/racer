#ifndef VEHICLE_POSITION_H_
#define VEHICLE_POSITION_H_

#include "racer/math/base_integrator.h"
#include "racer/math/primitives.h"

namespace racer {

    struct vehicle_position {
    private:
        racer::math::point location_;
        double heading_angle_;
        bool is_valid_;

    public:
        vehicle_position(double x, double y, double heading_angle)
            : location_{x, y}, heading_angle_{heading_angle}, is_valid_{true}
        {
        }

        vehicle_position(racer::math::point location, double heading_angle)
            : location_{location}, heading_angle_{heading_angle}, is_valid_{true}
        {
        }

        vehicle_position()
            : location_{0, 0}, heading_angle_{0}, is_valid_{false}
        {   
        }

        vehicle_position(const vehicle_position& copy_from) = default;
        vehicle_position& operator=(const vehicle_position& copy_from) = default;

        vehicle_position(vehicle_position&& pos) = default;
        vehicle_position& operator=(vehicle_position&& pos) noexcept {
            pos.is_valid_ = false;
            location_ = std::move(pos.location_);
            heading_angle_ = std::move(pos.heading_angle_);
            is_valid_ = true;
            return *this;
        }

        bool operator==(const vehicle_position& other) const noexcept {
            return location_ == other.location_ && heading_angle_ == other.heading_angle_;
        }

        bool is_valid() const noexcept {
            return is_valid_;
        }

        vehicle_position operator+(const vehicle_position& other) const noexcept {
            return vehicle_position(location_ + other.location_, heading_angle_ + other.heading_angle_);
        }

        const racer::math::vector& location() const noexcept { return location_; }
        constexpr const double& heading_angle() const noexcept { return heading_angle_; }

        vehicle_position integrate(const std::unique_ptr<racer::math::base_integrator>& integrator) const {
            return vehicle_position(
                { integrator->integrate(location_.x()), integrator->integrate(location_.y()) },
                integrator->integrate(heading_angle_)
            );
        }
    };

}

#endif
