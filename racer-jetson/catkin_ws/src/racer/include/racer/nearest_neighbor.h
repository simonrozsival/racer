#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>
#include <cassert>
#include <unordered_map>

#include "racer/math.h"

namespace racer::nearest_neighbor {

namespace kd_tree {

using vec = std::vector<racer::math::point>;
using point = racer::math::point;

enum splitting_axis { x, y };

struct node {
    const point pt;
    const std::size_t index;
    const splitting_axis axis;
    const std::unique_ptr<node> left_subtree;
    const std::unique_ptr<node> right_subtree;

    node(point pt, std::size_t index, splitting_axis axis, std::unique_ptr<node> left, std::unique_ptr<node> right)
        : pt{pt}, index{index}, axis{axis}, left_subtree{std::move(left)}, right_subtree{std::move(right)}
    {
    }
};

class tree
{
private:
    const std::unique_ptr<node> root_;

public:
    tree(vec points)
        : root_{build_subtree(points, 0, (long)(points.size() - 1), splitting_axis::x)}
    {
        assert(root_ != nullptr);
    }

    std::size_t find_nearest_neighbor(point pt) const
    {
        return find_nearest_in_subtree(root_, pt, {root_, pt}).index;
    }

private:
    // assumption: all the points in `data` are distinct
    std::unique_ptr<node> build_subtree(
        vec data,
        long from,
        long to,
        splitting_axis split_by) const
    {
        if (to < from)
        {
            return nullptr;
        }

        // find median
        long median = (from + to) / 2;
        std::partial_sort(
            data.begin() + from,
            data.begin() + median + 1,
            data.begin() + to + 1,
            split_by == splitting_axis::x ? &by_x : &by_y);

        // create subtrees recursively
        auto next_axis = split_by == splitting_axis::x ? splitting_axis::y : splitting_axis::x;
        auto left_subtree = build_subtree(data, from, median - 1, next_axis);
        auto right_subtree = build_subtree(data, median + 1, to, next_axis);

        // return the pointer
        return std::make_unique<node>(
            data[median],
            median,
            split_by,
            std::move(left_subtree),
            std::move(right_subtree));
    }

    struct nearest_so_far {
        std::size_t index;
        point pt;
        double distance_sq;

        nearest_so_far(const std::unique_ptr<node>& tree_node, point pt)
            : index{tree_node->index}, pt{tree_node->pt}, distance_sq{tree_node->pt.distance_sq(pt)}
        {
        }

        nearest_so_far(const nearest_so_far& other) = default;
        nearest_so_far& operator=(const nearest_so_far& other) = default;
        nearest_so_far(nearest_so_far&& other) = default;
        nearest_so_far& operator=(nearest_so_far&& other) = default;
    };

    nearest_so_far find_nearest_in_subtree(const std::unique_ptr<node>& subtree_root, point pt, nearest_so_far nearest) const {
        // end of recursion
        if (!subtree_root)
        {
            return nearest;
        }

        if (subtree_root->pt.distance_sq(pt) < nearest.distance_sq)
        {
            nearest = nearest_so_far(subtree_root, pt);
        }

        // have we found an (almost) exact match?
        if (nearest.distance_sq < 1e-3)
        {
            return nearest;
        }

        nearest = is_left_of(subtree_root, pt)
            ? find_nearest_in_subtree(subtree_root->left_subtree, pt, nearest)
            : find_nearest_in_subtree(subtree_root->right_subtree, pt, nearest);

        bool nearest_could_be_in_the_other_subtree = projected_distance_sq_between(subtree_root, pt) < nearest.distance_sq;
        if (nearest_could_be_in_the_other_subtree)
        {
            nearest = is_left_of(subtree_root, pt)
                ? find_nearest_in_subtree(subtree_root->right_subtree, pt, nearest)
                : find_nearest_in_subtree(subtree_root->left_subtree, pt, nearest);
        }

        return nearest;
    }

    inline double projected_distance_sq_between(const std::unique_ptr<node>& tree_node, point pt) const {
        const double d = tree_node->axis == splitting_axis::x ? tree_node->pt.x() - pt.x() : tree_node->pt.y() - pt.y();
        return d * d;
    }

    inline bool is_left_of(const std::unique_ptr<node>& tree_node, const point& pt) const {
        return tree_node->axis == splitting_axis::x ? by_x(pt, tree_node->pt) : by_y(pt, tree_node->pt);
    }

    static inline bool by_x(const point& a, const point& b) {
        return a.x() < b.x();
    }

    static inline bool by_y(const point& a, const point& b) {
        return a.y() < b.y();
    }
};

}

class linear_search {
private:
    const std::vector<racer::math::point> centers_;

public:
    linear_search(std::vector<racer::math::point> centers)
        : centers_{centers}
    {
    }
    
    std::size_t find_nearest_neighbor(const racer::math::point pt) const {
        std::size_t closest_circle = 0;
        double min_distance_sq = pt.distance_sq(centers_[0]);
        for (std::size_t i = 1; i < centers_.size(); ++i)
        {
            double distance_sq = pt.distance_sq(centers_[i]);
            if (distance_sq < min_distance_sq)
            {
                closest_circle = i;
                min_distance_sq = distance_sq;
            }
        }
        return closest_circle;
    }
};

class cached_linear_search {
private:
    const linear_search linear_;
    double cell_size_;
    mutable std::unordered_map<long, std::size_t> cache_;

public:
    cached_linear_search(std::vector<racer::math::point> centers)
        : linear_{centers}, cell_size_{0.5}
    {
    }
    
    std::size_t find_nearest_neighbor(const racer::math::point& pt) const {
        const auto index = index_of(pt);
        const auto search = cache_.find(index);
        if (search != cache_.end()) {
            return search->second;
        }

        std::cout << "miss" << std::endl;
        const auto result = linear_.find_nearest_neighbor(pt);
        cache_[index] = result;
        return result;
    }

private:
    inline long index_of(const racer::math::point& pt) const {
        return long(pt.x() / cell_size_) * 1000000 + long(pt.y() / cell_size_);
    }
};

}