#ifndef TRACK_ANALYSIS_H_
#define TRACK_ANALYSIS_H_

#include <vector>
#include <list>

#include "../math/primitives.h"
#include "../sehs/space_exploration.h"

#include "./occupancy_grid_collisions.h"

using namespace math;

namespace racing {

    class track_analysis {
    public:
        track_analysis(
            const occupancy_grid& grid,
            const double max_distance_between_waypoints,
            const int number_of_neighbors)
            : grid_(grid),
            max_distance_between_waypoints_(max_distance_between_waypoints),
            number_of_neighbors_(number_of_neighbors)
        {
        }

        const std::vector<point> find_apexes(
            const double vehicle_radius,
            const vehicle_position& initial_position,
            const std::list<point>& circuit_definition) const {

            sehs::space_exploration exploration(grid_, vehicle_radius, number_of_neighbors_);
            auto circle_path = exploration.explore_grid(initial_position, circuit_definition);

            if (circle_path.size() == 0) {
                return std::vector<point>(); // no path was found
            }

            std::vector<point> apex_waypoints;

            auto prev_circle = std::make_unique<circle>(circle_path.front());
            auto last_circle = std::make_unique<circle>(*prev_circle);

            for (const auto& next_step : circle_path) {
                const double dist = (last_circle->center - next_step.center).length_sq();
                if (dist >= pow(max_distance_between_waypoints_, 2) || !are_directly_visible(last_circle->center, next_step.center)) {
                    apex_waypoints.push_back(prev_circle->center);
                    last_circle = std::move(prev_circle);
                }

                prev_circle = std::make_unique<circle>(next_step);
            }

            // continue until the first apex is visible
            for (const auto& next_step : circle_path) {
                const double dist = (last_circle->center - next_step.center).length_sq();
                if (dist >= pow(max_distance_between_waypoints_, 2) || !are_directly_visible(last_circle->center, next_step.center)) {
                    apex_waypoints.push_back(prev_circle->center);
                    last_circle = std::move(prev_circle);

                    if (are_directly_visible(last_circle->center, apex_waypoints.front())) {
                        break;
                    }
                }

                prev_circle = std::make_unique<circle>(next_step);
            }


            return merge_close(apex_waypoints, 5 * vehicle_radius);
        }

    private:
        const occupancy_grid& grid_;
        const double max_distance_between_waypoints_;
        const int number_of_neighbors_;

        bool are_directly_visible(const point& a, const point& b) const {
            double distance = (a - b).length();
            double dx = grid_.cell_size * (b.x - a.x) / distance;
            double dy = grid_.cell_size * (b.y - a.y) / distance;

            double x = a.x;
            double y = a.y;

            while ((point(x, y) - b).length() >= grid_.cell_size) {
                x += dx;
                y += dy;

                int cx = int(x / grid_.cell_size);
                int cy = int(y / grid_.cell_size);

                if (grid_.is_occupied(cx, cy)) {
                    return false;
                }
            }

            return true;
        }

        const std::vector<point> merge_close(std::vector<point> original, double min_distance) const {
            std::vector<double> weights(original.size(), 1);
            std::vector<point> points{ original.begin(), original.end() };
            
            for (int i = 0; i < weights.size(); ++i) {
                int j = (i + 1) % weights.size();
                while (i != j && (points[i] - points[j]).length_sq() < pow(min_distance, 2)) {
                    points[i] = points[i].interpolate_with(points[j], weights[i], weights[j]);

                    // remove the second point
                    weights.erase(weights.begin() + j);
                    points.erase(points.begin() + j);

                    if (i > j) {
                        --i;
                    }

                    j = (i + 1) % weights.size();
                }
            }

            return points;
        }
    };
}

#endif
