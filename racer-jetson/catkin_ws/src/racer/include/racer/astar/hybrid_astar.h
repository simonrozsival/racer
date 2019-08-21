#ifndef HYBRID_ASTAR_H_
#define HYBRID_ASTAR_H_

#include "./discretized_astar_problem.h"

namespace racer::astar::hybrid_astar {
    struct discrete_state {
        const int x, y, heading, speed;

        discrete_state(int x, int y, int heading, int speed)
            : x(x), y(y), heading(heading), speed(speed)
        {
        }

        bool operator ==(const discrete_state& other) const {
            return x == other.x && y == other.y && heading == other.heading && speed == other.speed;
        }

        bool operator !=(const discrete_state& other) const {
            return !(*this == other);
        }
    };

    struct discretization : public racer::astar::discretization<discrete_state> {
    private:
        const double x, y, speed;
        const racer::math::angle heading;

    public:
        discretization(double x, double y, racer::math::angle heading, double speed)
            : x(x), y(y), heading(heading), speed(speed)
        {
        }

        std::unique_ptr<discrete_state> discretize(const state& state) const {
            return std::make_unique<discrete_state>(
                (int)floor(state.position.x / x),
                (int)floor(state.position.y / y),
                (int)floor(racer::math::angle(state.position.heading_angle) / heading),
                (int)floor(state.speed / speed));
        }

        bool is_ready() const override { return true; }
    };

}

namespace std {
    template<>
    struct hash<std::pair<racer::astar::hybrid_racer::astar::discrete_state, size_t>>
    {
        size_t operator()(const std::pair<racer::astar::hybrid_racer::astar::discrete_state, size_t>& obj) const
        {
            size_t seed = 0;

            racer::math::hash_combine(seed, obj.first.x);
            racer::math::hash_combine(seed, obj.first.y);
            racer::math::hash_combine(seed, obj.first.heading);
            racer::math::hash_combine(seed, obj.first.speed);
            racer::math::hash_combine(seed, obj.second);

            return seed;
        }
    };
}

#endif