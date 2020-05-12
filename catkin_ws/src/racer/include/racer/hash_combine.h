#include <iostream>

namespace std
{

template <>
struct hash<std::pair<int, int>>
{
  size_t operator()(const std::pair<int, int> &obj) const
  {
    size_t seed = 0;

    racer::math::hash_combine(seed, obj.first);
    racer::math::hash_combine(seed, obj.second);

    return seed;
  }
};
}  // namespace std
