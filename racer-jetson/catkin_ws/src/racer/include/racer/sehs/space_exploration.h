#ifndef SPACE_EXPLORATION_H_
#define SPACE_EXPLORATION_H_

#include <list>
#include <cmath>

#include "racer/astar/astar.h"
#include "racer/math/primitives.h"
#include "racer/circuit.h"

namespace std {
    template<>
    struct hash<std::pair<int, int>>
    {
        size_t operator()(const std::pair<int, int>& obj) const
        {
            size_t seed = 0;

            racer::math::hash_combine(seed, obj.first);
            racer::math::hash_combine(seed, obj.second);

            return seed;
        }
    };
}

namespace racer::sehs {

    struct circle_node {
        const racer::math::circle examined_circle;
        const double distance_from_start;
        const double distance_estimate;
        const double heading_angle;
        std::shared_ptr<circle_node> parent;

        circle_node(
            racer::math::circle examined_circle,
            double distance,
            double distance_to_go,
            double heading_angle,
            std::shared_ptr<circle_node> parent)
            : examined_circle(examined_circle),
            distance_from_start(distance),
            distance_estimate(distance + distance_to_go),
            heading_angle(heading_angle),
            parent(parent)
        {
        }

        std::list<racer::math::circle> reconstruct_path() {
            std::list<racer::math::circle> path;

            path.push_back(examined_circle);
            auto node = parent;

            while (node) {
                path.push_front(node->examined_circle);
                node = node->parent;
            }

            return path;
        }
    };

    struct distance_estimate {
        bool operator()(const std::shared_ptr<circle_node>& a, const std::shared_ptr<circle_node>& b) const {
            return a->distance_estimate > b->distance_estimate;
        }
    }; 

    class space_exploration {
    public:
        space_exploration(
            const racer::occupancy_grid& grid,
            const double vehicle_radius,
            const int number_of_expanded_points)
            : grid_(grid),
            min_radius_(vehicle_radius),
            max_radius_(4 * vehicle_radius),
            number_of_expanded_points_(number_of_expanded_points)
        {
        }

        const std::list<racer::math::circle> explore_grid(
            const racer::vehicle_position& initial_position,
            const std::list<racer::math::point>& waypoints) {

            std::list<racer::math::circle> path;

            path.push_back(generate_circle(initial_position.location()));
            double initial_heading_angle = initial_position.heading_angle;

            auto goal_it = waypoints.cbegin();

            while (goal_it != waypoints.cend()) {
                auto to_next_waypoint = find_path(path.back(), initial_heading_angle, *goal_it);

                if (to_next_waypoint.size() == 0) {
                    return std::list<racer::math::circle>(); // there is no path
                }

                // append the path to the next waypoint
                path.insert(path.end(), to_next_waypoint.begin(), to_next_waypoint.end());

                auto last = --path.end();
                auto last_but_one = --(--path.end());

                auto final_direction = last->center - last_but_one->center;
                initial_heading_angle = atan2(final_direction.y, final_direction.x);

                // move to the next waypoint
                ++goal_it;
            }

            return optimize_path(path);
        }

    private:
        const racer::occupancy_grid& grid_;
        const double min_radius_, max_radius_;
        const int number_of_expanded_points_;

        racer::math::circle generate_circle(racer::math::point point) const {
            double radius = grid_.distance_to_closest_obstacle(point, max_radius_);
            return racer::math::circle(point, radius);
        }

        const std::list<racer::math::circle> find_path(const racer::math::circle& from, double initial_heading_angle, const racer::math::point& to) {

            std::priority_queue<
                std::shared_ptr<circle_node>,
                std::vector<std::shared_ptr<circle_node>>,
                distance_estimate> open{ distance_estimate() };
            std::vector<racer::math::circle> closed;

            open.push(std::make_shared<circle_node>(from, 0, 0, initial_heading_angle, nullptr));

            while (open.size() > 0 && closed.size() < 5000) {
                if (open.top()->examined_circle.contains(to)) {
                    return open.top()->reconstruct_path();
                }

                auto nearest = open.top();
                open.pop();

                for (auto node : expand(nearest, to)) {
                    if (std::find(closed.begin(), closed.end(), node->examined_circle) == closed.end()) {
                        open.push(node);
                    }
                }

                closed.push_back(nearest->examined_circle);
            }

            
            std::cout << "space exploration was unsuccessful" << std::endl;
            std::cout << "failed after exploring " << closed.size() << " nodes" << std::endl;

            std::cout << "x,y,r" << std::endl;
            for (const auto c : closed) {
              std::cout << c.center.x << "," << c.center.y << "," << c.radius << std::endl;
            }

            // return std::list<racer::math::circle>{ closed.begin(), closed.end() };
            return std::list<racer::math::circle>();
        }

        std::list<std::shared_ptr<circle_node>> expand(const std::shared_ptr<circle_node>& node, const racer::math::point& goal) const {
            std::list<std::shared_ptr<circle_node>> nodes;
            const double sector_angle = (2.0 / 3.0) * pi;

            for (auto center : node->examined_circle.points_on_circumference(node->heading_angle - (sector_angle / 2), sector_angle, number_of_points(node->examined_circle.radius))) {
                auto circle = generate_circle(center);
                if (circle.radius >= min_radius_) {
                    double distance_estimate = (circle.center - goal).length();
                    auto direction = circle.center - node->examined_circle.center;
                    double heading_angle = atan2(direction.y, direction.x);

                    nodes.push_back(
                        std::make_shared<circle_node>(
                            circle,
                            node->distance_from_start + node->examined_circle.radius,
                            distance_estimate,
                            heading_angle,
                            node));
                }
            }

            return nodes;
        }

        int number_of_points(double radius) const {
            return int((1 + (radius - min_radius_) / (max_radius_ - min_radius_)) * number_of_expanded_points_);
        }

        racer::math::circle try_to_optimize(const racer::math::circle& a, const racer::math::circle& b, const racer::math::circle& c) const {
            racer::math::point interpolated_center = a.center.interpolate_with(c.center, a.radius, b.radius);
            const double radius = grid_.distance_to_closest_obstacle(interpolated_center, max_radius_);
            return radius >= b.radius ? racer::math::circle(interpolated_center, radius) : b;
        }

        const std::list<racer::math::circle> optimize_path(const std::list<racer::math::circle> original) const {
            std::vector<racer::math::circle> circles{ original.begin(), original.end() };
            std::vector<bool> can_be_optimized(original.size(), true);

            bool can_optimize = true;
            while (can_optimize) {
                can_optimize = false;
                for (int i = 1; i < can_be_optimized.size() - 1; ++i) {
                    if (can_be_optimized[i]) {
                        const auto optimized_circle = try_to_optimize(circles[i - 1], circles[i], circles[i + 1]);
                        if (optimized_circle.center != circles[i].center && optimized_circle.radius != circles[i].radius) {
                            circles[i] = optimized_circle;
                            can_be_optimized[i - 1] = true;
                            can_be_optimized[i + 1] = true;
                            can_optimize = true;
                        }
                    }
                }
            }

            std::list<racer::math::circle> optimized_path{ circles.begin(), circles.end() };
            return optimized_path;
        }
    };
}

#endif
