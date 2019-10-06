#ifndef CIRCUIT_H_
#define CIRCUIT_H_

#include <vector>

#include "racer/occupancy_grid.h"

namespace racer
{

class circuit
{
public:
    const size_t number_of_waypoints;
    const vehicle_position start_position;
    const occupancy_grid &grid;
    const std::vector<racer::math::point> waypoints;

private:
    const double waypoint_radius_sq_;
    std::vector<double> remaining_distances_;

public:
    circuit(
        vehicle_position start,
        std::vector<racer::math::point> list_of_waypoints,
        double waypoint_radius,
        const occupancy_grid &grid)
        : number_of_waypoints(list_of_waypoints.size()),
          start_position(start),
          grid(grid),
          waypoints(std::vector<racer::math::point>{list_of_waypoints.begin(), list_of_waypoints.end()}),
          waypoint_radius_sq_(waypoint_radius * waypoint_radius)
    {
        double remaining_distance = 0;
        remaining_distances_.reserve(waypoints.size());

        std::list<double> distances;
        for (size_t n = waypoints.size() - 1; n > 0; --n)
        {
            distances.push_front(remaining_distance);
            remaining_distance += (waypoints[n - 1]).distance(waypoints[n]);
        }

        distances.push_front(remaining_distance);

        std::copy(std::begin(distances), std::end(distances), std::back_inserter(remaining_distances_));
    }

    bool collides(const vehicle_position &position) const
    {
        return grid.collides(position.location().x(), position.location().y());
    }

    bool passes_waypoint(const vehicle_position &position, std::size_t waypoint_index) const
    {
        // I assume that waypoint_index is always in the bounds
        return distance_to_waypoint_sq(position.location(), waypoint_index) < waypoint_radius_sq_;
    }

    double distance_to_waypoint_sq(const math::vector& position, std::size_t n) const
    {
        if (n >= waypoints.size())
            return 0;

        return position.distance_sq(waypoints[n]);
    }

    double distance_to_waypoint(const math::vector& position, std::size_t n) const
    {
        return std::sqrt(distance_to_waypoint_sq(position, n));
    }

    double remaining_distance_estimate(std::size_t n) const
    {
        if (n >= waypoints.size())
            return 0;

        return remaining_distances_[n];
    }
};

} // namespace racer

#endif