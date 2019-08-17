#ifndef CIRCUIT_H_
#define CIRCUIT_H_

#include <vector>

#include "racing/collision_detection/occupancy_grid.h"

namespace racing {

    class circuit {
    public:
        const size_t number_of_waypoints;
        const vehicle_position start_position;
        const occupancy_grid& grid;
        const std::vector<math::point> waypoints;
        const double cell_size;

        circuit(
            double cell_size,
            vehicle_position start,
            std::vector<math::point> list_of_waypoints,
            double waypoint_radius,
            const occupancy_grid& grid)
            : cell_size(cell_size),
            start_position(start),
            waypoints(std::vector<math::point> { list_of_waypoints.begin(), list_of_waypoints.end() }),
            waypoint_radius_sq_(waypoint_radius * waypoint_radius),
            grid(grid),
            number_of_waypoints(list_of_waypoints.size())
        {
            double remaining_distance = 0;
            remaining_distances_.reserve(waypoints.size());

            std::list<double> distances;
            for (size_t n = waypoints.size() - 1; n > 0; --n) {
                distances.push_front(remaining_distance);
                remaining_distance += (waypoints[n - 1] - waypoints[n]).length();
            }

            distances.push_front(remaining_distance);

            std::copy(std::begin(distances), std::end(distances), std::back_inserter(remaining_distances_));
        }

        bool collides(const vehicle_position& position) const {
            return grid.collides(position.x, position.y);
        }

        bool passes_waypoint(const vehicle_position& position, size_t waypoint_index) const {
            // I assume that waypoint_index is always in the bounds
            const double dx = position.x - waypoints[waypoint_index].x;
            const double dy = position.y - waypoints[waypoint_index].y;
            return dx * dx + dy * dy < waypoint_radius_sq_;
        }

        double distance_to_waypoint(double x, double y, size_t n) const {
            if (n >= waypoints.size()) return 0;

            auto dx = x - waypoints[n].x;
            auto dy = y - waypoints[n].y;
            return sqrt(dx * dx + dy * dy);
        }

        double remaining_distance_estimate(size_t n) const {
            if (n >= waypoints.size()) return 0;

            return remaining_distances_[n];
        }
    private:
        const double waypoint_radius_sq_;
        
        std::vector<double> remaining_distances_;
    };

}

#endif