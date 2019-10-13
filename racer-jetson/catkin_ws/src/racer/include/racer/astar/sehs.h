#ifndef SEHS_H_
#define SEHS_H_

#include <vector>

#include "racer/astar/discretized_search_problem.h"

namespace racer::astar::sehs::kinematic
{

struct discrete_state
{
private:
    int circle_, heading_, rpm_;

    friend class std::hash<std::pair<racer::astar::sehs::kinematic::discrete_state, size_t>>;

public:
    discrete_state() : circle_{0}, heading_{0}, rpm_{0}
    {
    }

    discrete_state(int circle, int heading, int rpm)
        : circle_{circle}, heading_{heading}, rpm_{rpm}
    {
    }

    discrete_state(const discrete_state &other) = default;
    discrete_state &operator=(const discrete_state &other) = default;

    discrete_state(discrete_state &&other) = default;
    discrete_state &operator=(discrete_state &&other) = default;

    bool operator==(const discrete_state &other) const
    {
        return circle_ == other.circle_ && heading_ == other.heading_ && rpm_ == other.rpm_;
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
    discretization(
        const std::vector<racer::math::circle> &circles,
        std::size_t heading_bins,
        std::size_t rpm_bins,
        racer::vehicle_model::rpm max_rpm)
        : circles_{circles},
          motor_rpm_{max_rpm / double(rpm_bins)},
          heading_{2 * M_PI / double(heading_bins)}
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

        racer::math::hash_combine(seed, obj.first.circle_);
        racer::math::hash_combine(seed, obj.first.heading_);
        racer::math::hash_combine(seed, obj.first.rpm_);
        racer::math::hash_combine(seed, obj.second);

        return seed;
    }
};
} // namespace std

#endif
