#pragma once

#include <vector>

#include "racer/sehs/nearest_neighbor.h"

namespace racer::astar::sehs
{

struct discrete_state
{
private:
  int circle_, heading_, rpm_;

  friend class std::hash<
      std::pair<racer::astar::sehs::discrete_state, size_t>>;

public:
  discrete_state() : circle_{0}, heading_{0}, rpm_{0} {}

  discrete_state(int circle, int heading, int rpm)
      : circle_{circle}, heading_{heading}, rpm_{rpm} {}

  discrete_state(const discrete_state &other) = default;
  discrete_state &operator=(const discrete_state &other) = default;

  discrete_state(discrete_state &&other) = default;
  discrete_state &operator=(discrete_state &&other) = default;

  bool operator==(const discrete_state &other) const
  {
    return circle_ == other.circle_ && heading_ == other.heading_ &&
            rpm_ == other.rpm_;
  }

  bool operator!=(const discrete_state &other) const
  {
    return !(*this == other);
  }
};
} // namespace racer::astar::sehs::kinematic

namespace std
{

template <>
struct hash<std::pair<racer::astar::sehs::discrete_state, size_t>>
{
  size_t operator()(
      const std::pair<racer::astar::sehs::discrete_state, size_t>
          &obj) const
  {
    size_t seed = 0;

    racer::math::hash_combine(seed, obj.first.circle_);
    racer::math::hash_combine(seed, obj.first.heading_);
    racer::math::hash_combine(seed, obj.first.rpm_);
    racer::math::hash_combine(seed, obj.second);

    return seed;
  }
};

} // namespace std
