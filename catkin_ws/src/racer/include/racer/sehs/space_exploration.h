#pragma once

#include <algorithm>
#include <filesystem>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "racer/astar/astar.h"
#include "racer/math.h"
#include "racer/track/occupancy_grid.h"
#include "racer/vehicle/configuration.h"

namespace racer::sehs
{
struct circle_node
{
  const racer::math::circle examined_circle;
  const double distance_from_start;
  const double distance_estimate;
  const double heading_angle;
  std::shared_ptr<circle_node> parent;

  circle_node(racer::math::circle examined_circle, double distance, double distance_to_go, double heading_angle,
              std::shared_ptr<circle_node> parent)
    : examined_circle(examined_circle)
    , distance_from_start(distance)
    , distance_estimate(distance + distance_to_go)
    , heading_angle(heading_angle)
    , parent(parent)
  {
  }

  std::vector<racer::math::circle> reconstruct_path()
  {
    std::vector<racer::math::circle> path;

    path.push_back(examined_circle);
    auto node = parent;

    while (node)
    {
      path.push_back(node->examined_circle);
      node = node->parent;
    }

    std::reverse(std::begin(path), std::end(path));
    return path;
  }
};

struct distance_estimate
{
  bool operator()(const std::shared_ptr<circle_node> &a, const std::shared_ptr<circle_node> &b) const
  {
    return a->distance_estimate > b->distance_estimate;
  }
};

class space_exploration
{
public:
  space_exploration(const double min_radius, const double max_radius, const int number_of_expanded_points)
    : min_radius_(min_radius), max_radius_(max_radius), number_of_expanded_points_(number_of_expanded_points)
  {
  }

  const std::vector<racer::math::circle> explore_grid(const std::shared_ptr<racer::track::occupancy_grid> grid,
                                                      const racer::vehicle::configuration &initial_position,
                                                      const std::vector<racer::math::point> &waypoints) const
  {
    std::vector<racer::math::circle> path;

    if (!all_waypoints_are_accessible(grid, waypoints))
    {
      return {};
    }

    path.push_back(generate_circle_around(initial_position.location(), grid));
    double initial_heading_angle = initial_position.heading_angle();

    auto goal_it = waypoints.cbegin();

    while (goal_it != waypoints.cend())
    {
      auto to_next_waypoint = find_path(path.back(), initial_heading_angle, *goal_it, grid);

      if (to_next_waypoint.size() == 0)
      {
        return {};  // there is no path
      }

      // append the path to the next waypoint
      path.insert(path.end(), to_next_waypoint.begin(), to_next_waypoint.end());

      auto last = --path.end();
      auto last_but_one = --(--path.end());

      auto final_direction = last->center() - last_but_one->center();
      initial_heading_angle = final_direction.angle();

      // move to the next waypoint
      ++goal_it;
    }

    return optimize_path(path, grid);
  }

private:
  const double min_radius_, max_radius_;
  const int number_of_expanded_points_;

  racer::math::circle generate_circle_around(racer::math::point point,
                                             const std::shared_ptr<racer::track::occupancy_grid> grid) const
  {
    double radius = grid->distance_to_closest_obstacle(point, max_radius_);
    return { point, radius };
  }

  const std::vector<racer::math::circle> find_path(const racer::math::circle &from, double initial_heading_angle,
                                                   const racer::math::point &to,
                                                   const std::shared_ptr<racer::track::occupancy_grid> grid) const
  {
    std::priority_queue<std::shared_ptr<circle_node>, std::vector<std::shared_ptr<circle_node>>, distance_estimate>
        open{ distance_estimate() };
    std::vector<racer::math::circle> closed;

    open.push(std::make_shared<circle_node>(from, 0, 0, initial_heading_angle, nullptr));

    while (open.size() > 0 && closed.size() < 100000)
    {
      if (open.top()->examined_circle.contains(to))
      {
        // success!
        return open.top()->reconstruct_path();
      }

      auto nearest = open.top();
      open.pop();

      for (auto node : expand(nearest, to, grid))
      {
        if (std::find(closed.begin(), closed.end(), node->examined_circle) == closed.end())
        {
          open.push(node);
        }
      }

      closed.push_back(nearest->examined_circle);
    }

    std::cerr << "space exploration failed after exploring " << closed.size() << " nodes between [" << from.center().x()
              << ", " << from.center().y() << "] and [" << to.x() << ", " << to.y() << "]" << std::endl;

    return {};
  }

  std::vector<std::shared_ptr<circle_node>> expand(const std::shared_ptr<circle_node> &node,
                                                   const racer::math::point &goal,
                                                   const std::shared_ptr<racer::track::occupancy_grid> grid) const
  {
    std::vector<std::shared_ptr<circle_node>> nodes;
    const double sector_angle = 2 * M_PI;  // this could be changed to narrow the search direction

    for (auto center : node->examined_circle.points_on_circumference(node->heading_angle - (sector_angle / 2),
                                                                     sector_angle, number_of_expanded_points_))
    {
      auto circle = generate_circle_around(center, grid);
      if (circle.radius() >= min_radius_)
      {
        double distance_estimate = circle.center().distance(goal);
        auto direction = circle.center() - node->examined_circle.center();
        double heading_angle = direction.angle();

        nodes.push_back(std::make_shared<circle_node>(circle,
                                                      node->distance_from_start + node->examined_circle.radius(),
                                                      distance_estimate, heading_angle, node));
      }
    }

    return nodes;
  }

  const std::vector<racer::math::circle> optimize_path(std::vector<racer::math::circle> &circles,
                                                       const std::shared_ptr<racer::track::occupancy_grid> grid) const
  {
    std::vector<bool> can_be_optimized(circles.size(), true);

    bool can_optimize;
    do
    {
      can_optimize = false;
      for (std::size_t i = 1; i < can_be_optimized.size() - 1; ++i)
      {
        if (can_be_optimized[i])
        {
          const auto optimized_circle = try_to_optimize(circles[i - 1], circles[i], circles[i + 1], grid);
          if (optimized_circle.center() != circles[i].center() && optimized_circle.radius() != circles[i].radius())
          {
            circles[i] = optimized_circle;
            can_be_optimized[i - 1] = true;
            can_be_optimized[i + 1] = true;
            can_optimize = true;
          }
        }
      }
    } while (can_optimize);

    return circles;
  }

  racer::math::circle try_to_optimize(const racer::math::circle &a, const racer::math::circle &b,
                                      const racer::math::circle &c,
                                      const std::shared_ptr<racer::track::occupancy_grid> grid) const
  {
    racer::math::point interpolated_center = a.center().interpolate_with(c.center(), a.radius(), b.radius());
    const double radius = grid->distance_to_closest_obstacle(interpolated_center, max_radius_);
    return radius >= b.radius() ? racer::math::circle(interpolated_center, radius) : b;
  }

  bool all_waypoints_are_accessible(const std::shared_ptr<racer::track::occupancy_grid> grid,
                                    const std::vector<racer::math::point> &waypoints) const
  {
    bool all_are_accessible = true;
    for (auto wp : waypoints)
    {
      if (grid->collides(wp))
      {
        std::cerr << "Waypoint " << wp << " is not accessible." << std::endl;
        all_are_accessible = false;
      }
    }

    return all_are_accessible;
  }
};
}  // namespace racer::sehs
