#ifndef HYBRID_ASTAR_H_
#define HYBRID_ASTAR_H_

#include "./discretized_astar_problem.h"

namespace astar {
    namespace hybrid_astar {
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

        struct discretization : public astar::discretization<discrete_state> {
        private:
            const double x, y, speed;
            const math::angle heading;

        public:
            discretization(double x, double y, math::angle heading, double speed)
                : x(x), y(y), heading(heading), speed(speed)
            {
            }

            std::unique_ptr<discrete_state> discretize(const state& state) const {
                return std::make_unique<discrete_state>(
                    (int)floor(state.position.x / x),
                    (int)floor(state.position.y / y),
                    (int)floor(math::angle(state.position.heading_angle) / heading),
                    (int)floor(state.speed / speed));
            }
        };

    }
}

namespace std {
    template<>
    struct hash<std::pair<astar::hybrid_astar::discrete_state, size_t>>
    {
        size_t operator()(const std::pair<astar::hybrid_astar::discrete_state, size_t>& obj) const
        {
            size_t seed = 0;

            math::hash_combine(seed, obj.first.x);
            math::hash_combine(seed, obj.first.y);
            math::hash_combine(seed, obj.first.heading);
            math::hash_combine(seed, obj.first.speed);
            math::hash_combine(seed, obj.second);

            return seed;
        }
    };
}

#endif