#ifndef SEHS_H_
#define SEHS_H_

#include <vector>

#include "racer/astar/discretized_astar_problem.h"

namespace racer::astar::sehs::kinematic
{

struct discrete_state
{
    int circle, heading, speed;

    discrete_state() : circle(0), heading(0), speed(0)
    {
    }

    discrete_state(int circle, int heading, int speed)
        : circle(circle), heading(heading), speed(speed)
    {
    }

    discrete_state(const discrete_state &other) = default;
    discrete_state &operator=(const discrete_state &other) = default;

    discrete_state(discrete_state &&other) = default;
    discrete_state &operator=(discrete_state &&other) = default;

    bool operator==(const discrete_state &other) const
    {
        return circle == other.circle && heading == other.heading && speed == other.speed;
    }

    bool operator!=(const discrete_state &other) const
    {
        return !(*this == other);
    }
};

struct discretization
    : public racer::astar::discretization<discrete_state, racer::vehicle_model::kinematic::state>
{
private:
    std::vector<racer::math::circle> circles_;
    const double motor_rpm_;
    const racer::math::angle heading_;

public:
    discretization(const std::vector<racer::math::circle> &circles, double heading, double motor_rpm)
        : circles_{circles},
          motor_rpm_(motor_rpm),
          heading_(heading)
    {
    }

    discrete_state operator()(const racer::vehicle_model::kinematic::state &state) const override
    {
        int closest_circle = 0;
        double min_distance_sq;
        for (std::size_t i = 0; i < circles_.size(); ++i)
        {
            double distance_sq = state.position().distance_sq(circles_[i].center());
            if (i == 0 || distance_sq < min_distance_sq)
            {
                closest_circle = i;
                min_distance_sq = distance_sq;
            }
        }

        return {
            closest_circle,
            (int)floor(racer::math::angle(state.configuration().heading_angle()) / heading_),
            (int)floor(state.motor_rpm() / motor_rpm_)};
    }
};

} // namespace racer::astar::sehs::kinematic

namespace std
{
template <>
struct hash<std::pair<racer::astar::sehs::kinematic::discrete_state, size_t>>
{
    size_t operator()(const std::pair<racer::astar::sehs::kinematic::discrete_state, size_t> &obj) const
    {
        size_t seed = 0;

        racer::math::hash_combine(seed, obj.first.circle);
        racer::math::hash_combine(seed, obj.first.heading);
        racer::math::hash_combine(seed, obj.first.speed);
        racer::math::hash_combine(seed, obj.second);

        return seed;
    }
};
} // namespace std

#endif
