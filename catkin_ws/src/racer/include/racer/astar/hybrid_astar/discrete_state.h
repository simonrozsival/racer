#pragma once

namespace racer::astar::hybrid_astar
{

struct discrete_state
{
    int x, y, heading, speed;

    discrete_state() : x(0), y(0), heading(0), speed(0)
    {
    }

    discrete_state(int x, int y, int heading, int speed)
        : x(x), y(y), heading(heading), speed(speed)
    {
    }

    discrete_state(const discrete_state &state) = default;
    discrete_state &operator=(const discrete_state &state) = default;

    discrete_state(discrete_state &&state) = default;
    discrete_state &operator=(discrete_state &&state) = default;

    bool operator==(const discrete_state &other) const
    {
        return x == other.x && y == other.y && heading == other.heading && speed == other.speed;
    }

    bool operator!=(const discrete_state &other) const
    {
        return !(*this == other);
    }
};

} // namespace racer::astar::hybrid_astar

namespace std
{
template <>
struct hash<std::pair<racer::astar::hybrid_astar::discrete_state, size_t>>
{
    size_t operator()(const std::pair<racer::astar::hybrid_astar::discrete_state, size_t> &obj) const
    {
        size_t seed = 0;

        racer::math::hash_combine(seed, obj.first.x);
        racer::math::hash_combine(seed, obj.first.y);
        racer::math::hash_combine(seed, obj.first.heading);
        racer::math::hash_combine(seed, obj.first.speed);
        racer::math::hash_combine(seed, obj.second);

        return seed;
    }
};

} // namespace std
