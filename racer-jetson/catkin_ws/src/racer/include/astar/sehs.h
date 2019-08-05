#ifndef SEHS_H_
#define SEHS_H_

#include <vector>
#include <list>

#include "./discretized_astar_problem.h"
#include "../sehs/space_exploration.h"

namespace astar {
    namespace sehs {

        struct discrete_state {
            const int circle, heading, speed;

            discrete_state(int circle, int heading, int speed)
                : circle(circle), heading(heading), speed(speed)
            {
            }

            bool operator ==(const discrete_state& other) const {
                return circle == other.circle && heading == other.heading && speed == other.speed;
            }

            bool operator !=(const discrete_state& other) const {
                return !(*this == other);
            }
        };

        struct discretization : public astar::discretization<discrete_state> {
        private:
            std::vector<math::circle> circles_;
            const double speed_;
            const math::angle heading_;

            double vehicle_radius_;
            int number_of_expanded_points_;
            bool is_initialized_;

        public:
            discretization(double vehicle_radius, int number_of_expanded_points, math::angle heading, double speed)
                : vehicle_radius_(vehicle_radius), number_of_expanded_points_(number_of_expanded_points), heading_(heading), speed_(speed), is_initialized_(false)
            {
            }

            bool is_ready() const override {
                return is_initialized_;
            }

            std::unique_ptr<discrete_state> discretize(const state& state) const {
                int closest_circle = 0;
                double min_distance_sq;
                for (int i = 0; i < circles_.size(); ++i) {
                    double distance_sq = (state.position.location() - circles_[i].center).length_sq();
                    if (i == 0 || distance_sq < min_distance_sq) {
                        closest_circle = i;
                        min_distance_sq = distance_sq;
                    }
                }

                return std::make_unique<discrete_state>(
                    closest_circle,
                    (int)floor(math::angle(state.position.heading_angle) / heading_),
                    (int)floor(state.speed / speed_));
            }

            void explore_grid(
                const racing::occupancy_grid& grid,
                const racing::vehicle_position& initial_position,
                const std::list<math::point>& waypoints) {

                circles_.clear();

                ::sehs::space_exploration exploration(grid, vehicle_radius_, number_of_expanded_points_);
                const auto circle_path = exploration.explore_grid(initial_position, waypoints);

                for (const auto circle : circle_path) {
                    circles_.push_back(circle);
                }

                is_initialized_ = true;
            }
        };

    }
}

namespace std {
    template<>
    struct hash<std::pair<astar::sehs::discrete_state, size_t>>
    {
        size_t operator()(const std::pair<astar::sehs::discrete_state, size_t>& obj) const
        {
            size_t seed = 0;

            math::hash_combine(seed, obj.first.circle);
            math::hash_combine(seed, obj.first.heading);
            math::hash_combine(seed, obj.first.speed);
            math::hash_combine(seed, obj.second);

            return seed;
        }
    };
}
#endif
