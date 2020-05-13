#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <memory>
#include <cassert>
#include <unordered_map>

#include "racer/math/point.h"

namespace racer::astar::sehs {

class nearest_neighbor {
private:
    const std::vector<racer::math::point> centers_;

public:
    nearest_neighbor(std::vector<racer::math::point> centers)
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

    std::size_t size() const { return centers_.size(); }
};

}