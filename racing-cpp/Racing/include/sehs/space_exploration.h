#ifndef SPACE_EXPLORATION_H_
#define SPACE_EXPLORATION_H_

#include <list>
#include <cmath>

#include "../astar/astar.h"
#include "../math/primitives.h"
#include "../racing/circuit.h"

namespace std {
    template<>
    struct hash<std::pair<int, int>>
    {
        size_t operator()(const std::pair<int, int>& obj) const
        {
            size_t seed = 0;

            math::hash_combine(seed, obj.first);
            math::hash_combine(seed, obj.second);

            return seed;
        }
    };
}

namespace sehs {

    struct circle_node {
        const math::circle examined_circle;
        const double distance_from_start;
        const double distance_estimate;
        const double heading_angle;
        std::shared_ptr<circle_node> parent;

        circle_node(
            math::circle examined_circle,
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

        std::list<math::circle> reconstruct_path() {
            std::list<math::circle> path;

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
            const racing::occupancy_grid& grid,
            const double vehicle_radius,
            const int number_of_expanded_points)
            : grid_(grid),
            min_radius_(vehicle_radius),
            max_radius_(4 * vehicle_radius),
            number_of_expanded_points_(number_of_expanded_points)
        {
        }

        const std::list<math::circle> explore_grid(
            const racing::vehicle_position& initial_position,
            const std::list<math::point>& waypoints) {

            std::list<math::circle> path;

            path.push_back(generate_circle(initial_position.location()));
            double initial_heading_angle = initial_position.heading_angle;

            auto goal_it = ++waypoints.cbegin();

            while (goal_it != waypoints.cend()) {
                auto to_next_waypoint = find_path(path.back(), initial_heading_angle, *goal_it);

                if (to_next_waypoint.size() == 0) {
                    return std::list<math::circle>(); // there is no path
                }

                std::cout << "found path between a pair of waypoints: " << path.back().center.x << "," << path.back().center.y << "->" << goal_it->x << "," << goal_it->y << std::endl;

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
        const racing::occupancy_grid& grid_;
        const double min_radius_, max_radius_;
        const int number_of_expanded_points_;

        math::circle generate_circle(math::point point) const {
            double radius = grid_.distance_to_closest_obstacle(point, max_radius_);
            return math::circle(point, radius);
        }

        const std::list<math::circle> find_path(const math::circle& from, double initial_heading_angle, const math::point& to) {

            std::priority_queue<
                std::shared_ptr<circle_node>,
                std::vector<std::shared_ptr<circle_node>>,
                distance_estimate> open{ distance_estimate() };
            std::vector<math::circle> closed;

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

            // return std::list<math::circle>{ closed.begin(), closed.end() };
            return std::list<math::circle>();
        }

        std::list<std::shared_ptr<circle_node>> expand(const std::shared_ptr<circle_node>& node, const math::point& goal) const {
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

        math::circle try_to_optimize(const math::circle& a, const math::circle& b, const math::circle& c) const {
            math::point interpolated_center = a.center.interpolate_with(c.center, a.radius, b.radius);
            const double radius = grid_.distance_to_closest_obstacle(interpolated_center, max_radius_);
            return radius >= b.radius ? math::circle(interpolated_center, radius) : b;
        }

        const std::list<math::circle> optimize_path(const std::list<math::circle> original) const {
            std::vector<math::circle> circles{ original.begin(), original.end() };
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

            std::list<math::circle> optimized_path{ circles.begin(), circles.end() };
            return optimized_path;
        }
    };
}

#endif
