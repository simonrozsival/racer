#pragma once

#include "racer/astar/discretized/base_discretization.h"
#include "racer/vehicle/kinematic/state.h"

#include "racer/astar/hybrid_astar/discrete_state.h"

namespace racer::astar::hybrid_astar
{

struct discretization
    : public racer::astar::discretized::base_discretization<discrete_state, racer::vehicle::kinematic::state>
{
private:
    const double x_, y_, motor_rpm_;
    const racer::math::angle heading_;

public:
    discretization(double x, double y, racer::math::angle heading, double motor_rpm)
        : x_(x), y_(y), motor_rpm_(motor_rpm), heading_(heading)
    {
    }

    discrete_state operator()(const racer::vehicle::kinematic::state &state) override
    {
        return {
            (int)floor(state.position().x() / x_),
            (int)floor(state.position().y() / y_),
            (int)floor(state.cfg().heading_angle() / heading_),
            (int)floor(state.motor_rpm() / motor_rpm_)};
    }

    std::string description() const override
    {
        std::stringstream s;
        s << "x-" << x_ << "_y-" << x_ << "-rpm-" << motor_rpm_ << "-theta-" << heading_.to_degrees();
        return s.str();
    }
};

} // namespace racer::astar::hybrid_astar
