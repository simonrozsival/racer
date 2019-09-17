#ifndef TRACK_ANALYSIS_H_
#define TRACK_ANALYSIS_H_

#include <vector>
#include <list>
#include <set>
#include <utility>
#include <algorithm>

#include "racer/math/primitives.h"
#include "racer/sehs/space_exploration.h"
#include "racer/occupancy_grid.h"

using namespace racer::math;

namespace racer
{

class track_analysis
{
public:
    track_analysis(
        const occupancy_grid &grid,
        const double max_distance_between_waypoints,
        const int number_of_neighbors)
        : grid_(grid),
          max_distance_between_waypoints_(max_distance_between_waypoints),
          number_of_neighbors_(number_of_neighbors)
    {
    }

    const std::list<point> find_corners(
        const double vehicle_radius,
        const vehicle_position &initial_position,
        const std::list<point> &circuit_definition) const
    {
        sehs::space_exploration exploration(grid_, vehicle_radius, 2 * vehicle_radius, number_of_neighbors_);
        auto circle_path = exploration.explore_grid(initial_position, circuit_definition);

        if (circle_path.size() == 0)
        {
            return std::list<point>(); // no path was found
        }

        std::vector<point> apex_waypoints;

        auto prev_circle = std::make_unique<circle>(circle_path.front());
        auto last_circle = std::make_unique<circle>(*prev_circle);

        // first iteration
        for (const auto &next_step : circle_path)
        {
            if (!are_directly_visible(last_circle->center, next_step.center))
            {
                apex_waypoints.push_back(prev_circle->center);
                last_circle = std::move(prev_circle);
            }

            prev_circle = std::make_unique<circle>(next_step);
        }

        // continue from the beginning
        for (const auto &next_step : circle_path)
        {
            if (next_step.center == apex_waypoints.front())
            {
                break;
            }

            if (!are_directly_visible(last_circle->center, next_step.center))
            {
                apex_waypoints.push_back(prev_circle->center);
                last_circle = std::move(prev_circle);
            }

            prev_circle = std::make_unique<circle>(next_step);
        }

        return merge_close(apex_waypoints, 10 * vehicle_radius);
    }

private:
    const occupancy_grid &grid_;
    const double max_distance_between_waypoints_;
    const int number_of_neighbors_;

    bool are_directly_visible(const point &a, const point &b) const
    {
        double distance = (a - b).length();
        double dx = grid_.cell_size * (b.x - a.x) / distance;
        double dy = grid_.cell_size * (b.y - a.y) / distance;

        double x = a.x;
        double y = a.y;

        while ((point(x, y) - b).length() >= grid_.cell_size)
        {
            x += dx;
            y += dy;

            if (grid_.collides(x, y))
            {
                return false;
            }
        }

        return true;
    }

    class priority_queue
    {
    private:
        std::vector<std::size_t> key_to_index;
        std::vector<std::pair<std::size_t, std::size_t>> heap; // max heap

    public:
        priority_queue(std::size_t n)
        {
            for (std::size_t i = 0; i < n; ++i)
            {
                key_to_index.push_back(i);
                heap.emplace_back(0, i); // all values are set to 0 initially
            }
        }

        const std::size_t peek_value() const
        {
            return heap[0].first;
        }

        const std::size_t peek_key() const
        {
            return heap[0].second;
        }

        const std::size_t value_of(std::size_t key) const
        {
            return heap[key_to_index[key]].first;
        }

        const void change_value(std::size_t key, std::size_t value)
        {
            const std::size_t original_value = value_of(key);
            if (original_value == value)
                return;

            std::size_t i = key_to_index[key];
            heap[i].first = value;

            if (original_value > value)
            {
                // bubble down - always swap with the bigger of its children
                while (left(i) < heap.size())
                {
                    std::size_t bigger_child = right(i) < heap.size() && heap[right(i)].first > heap[left(i)].first ? right(i) : left(i);
                    if (heap[i].first >= heap[bigger_child].first)
                        break;

                    swap(i, bigger_child);
                    i = bigger_child;
                }
            }
            else
            {
                // bubble up - swap with the parent as long as it is smaller and we haven't reached top
                while (i > 0 && heap[i].first > heap[parent(i)].first)
                {
                    std::size_t p_i = parent(i);
                    swap(i, p_i);
                    i = p_i;
                }
            }
        }

    private:
        inline const std::size_t parent(std::size_t i) const { return i / 2; }
        inline const std::size_t left(std::size_t i) const { return i * 2; }
        inline const std::size_t right(std::size_t i) const { return i * 2 + 1; }

        void swap(std::size_t i, std::size_t j)
        {
            if (i == j)
                return;

            std::size_t key_i = heap[i].second;
            std::size_t key_j = heap[j].second;

            key_to_index[key_j] = i;
            key_to_index[key_i] = j;

            auto tmp = heap[i];
            heap[i].first = heap[j].first;
            heap[j].first = tmp.first;

            heap[i].second = key_j;
            heap[j].second = key_i;
        }
    };

    const std::list<point> merge_close(std::vector<point> points, double d_min) const
    {
        std::vector<bool> used(points.size(), true); // all points are used at the beginning
        track_analysis::priority_queue queue(points.size());

        for (std::size_t i = 0; i < points.size(); ++i)
        {
            const auto close = close_points(points, used, i, d_min);
            queue.change_value(i, close.size());
        }

        while (queue.peek_value() > 0)
        {
            const std::size_t i_max = queue.peek_key();
            const auto close = close_points(points, used, i_max, d_min);

            std::set<std::size_t> to_remove{close.begin(), close.end()};
            to_remove.insert(i_max);

            const auto i_mid = middle_index(to_remove, used);

            to_remove.erase(i_mid);

            // remove all the points except for the mid one
            for (std::size_t i : to_remove)
            {
                queue.change_value(i, 0);
                used[i] = false;
            }

            // update the numbers of close points
            for (std::size_t i : to_remove)
            {
                const auto remaining_close = close_points(points, used, i, d_min);
                for (const auto &close : remaining_close)
                {
                    queue.change_value(close, queue.value_of(close) - 1);
                }
            }
        }

        std::list<point> remaining;
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            if (used[i])
            {
                remaining.push_back(points[i]);
            }
        }

        return remaining;
    }

    const std::set<std::size_t> close_points(const std::vector<point> &points, const std::vector<bool> &used, std::size_t i, double d_min) const
    {
        std::set<std::size_t> close;

        std::size_t j = (i + 1) % points.size();
        while (j != i && (used[j] == false || points[i].distance_sq(points[j]) < pow(d_min, 2)))
        {
            auto res = close.insert(j);
            if (!res.second)
            {
                break;
            }

            j = (j + 1) % points.size();
        }

        j = i == 0 ? points.size() : i - 1;
        while (j != i && (used[j] == false || points[i].distance_sq(points[j]) < pow(d_min, 2)))
        {
            auto res = close.insert(j);
            if (!res.second)
            {
                break;
            }

            j = j == 0 ? points.size() : j - 1;
        }

        return close;
    }

    const std::size_t middle_index(const std::set<std::size_t> &indices, const std::vector<bool> &used) const
    {
        std::vector<std::size_t> sequence{indices.begin(), indices.end()};
        std::sort(sequence.begin(), sequence.end());

        // iterate over the sequence and check, that there is no gap
        // if there is a gap, the start is the first element after the gap
        // otherwise it is the one with the smallest index
        std::size_t skipped_removed_points = 0;
        std::size_t i_start = 0;
        for (std::size_t i = 1; i < used.size(); ++i)
        {
            if (sequence[i] != sequence[i - 1] + 1)
            {
                // what index do we expect to be next?
                std::size_t j = (sequence[0] + i + skipped_removed_points) % used.size();

                // either we're skipping or there is a gap
                if (used[j])
                {
                    // a gap!
                    i_start = sequence[i];
                    break;
                }
                else
                {
                    ++skipped_removed_points;
                }
            }
        }

        return sequence[(std::size_t)std::floor(((i_start + sequence.size()) / 2) % sequence.size())];
    }
};
} // namespace racer

#endif
